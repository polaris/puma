#ifndef CLOCK_SYNC_H
#define CLOCK_SYNC_H

#include "netint.h"
#include "sync_message.h"
#include "clock_helper.h"
#include "servo.h"
#include "timestamped_socket.h"

#include <asio.hpp>
#include <chrono>
#include <functional>
#include <optional>
#include <random>

namespace clocksync {

enum class Role { Master, Slave };

struct Stats {
    double offset        = 0.0;   // master - local, seconds
    double pathDelay     = 0.0;   // rolling minimum one-way delay, seconds
    // Empty until the servo's delay window holds enough samples for the gate to
    // be applied. A threshold is computed from the first sample on, but until
    // the window fills nothing is rejected by it, so reporting it would read as
    // a live rejection threshold that is not in force.
    std::optional<double> gateThreshold;  // current outlier gate, seconds
    double quality       = 0.0;   // 0..1, 1 is best

    // Which receive timestamps the servo is actually being fed. A pair of nodes
    // where only one side has kernel stamps is structurally asymmetric, so this
    // belongs in the diagnostics rather than being inferred from a odd offset.
    bool   kernelTimestamps = false;
    double kernelLag        = 0.0;   // seconds of receive-path latency removed

    // Per-node signature, from our own loopback datagrams. theta between two
    // nodes is the difference of their signatures, so receiver-to-receiver
    // agreement is limited by how much these differ, not by how large either
    // one is. Observable before any of it reaches the audio path.
    double txProbeFloor     = 0.0;   // seconds, best (transmit + loopback)
    std::uint64_t txProbes  = 0;

    // Per-leg floors. floorA - floorB is twice the systematic offset the
    // two-way exchange cannot detect.
    double floorA           = 0.0;   // seconds, master -> slave
    double floorB           = 0.0;   // seconds, slave  -> master

    std::uint64_t rejected  = 0;  // dropped by the delay gate
    std::uint64_t tooSoon   = 0;  // sample interval implausibly short
    std::uint64_t unmatched = 0;  // DelayResp with no matching request
    std::uint64_t noSync    = 0;  // DelayResp before any Sync
    std::uint64_t staleSync = 0;  // paired Sync too old to use
};

// One completed DelayReq/DelayResp exchange, before the servo smooths it.
// Handed to the sample callback on the io thread, so the callback must not
// block. The smoothed, thread-safe view of the same clock is Stats.
struct Sample {
    double offset;        // (t2-t1 - (t4-t3))/2, seconds, this exchange alone
    double delay;         // one-way delay for this exchange, seconds
    double mappedOffset;  // the servo's estimate of offset; sign follows the
                          // field above, so it is -Stats::offset
    double skew;          // master seconds per local second, minus 1

    std::uint64_t rejected;  // servo counters at the time of this sample
    std::uint64_t tooSoon;
};

struct Config {
    asio::ip::udp::endpoint group;        // 239.255.0.2:12346 — not the audio group
    net::Interface iface;                 // Network interface
    std::uint8_t domain = 0;              // two systems, one LAN, no interference
    std::uint64_t nodeId = 0;             // used to ignore our own packets
    std::chrono::milliseconds syncInterval{125};        // 8/s
    std::chrono::milliseconds delayReqInterval{125};    // 8/s
    double acquireBandwidth = kDefaultAcquireBandwidth; // Hz
    double lockBandwidth    = kDefaultLockBandwidth;    // Hz
    bool loopback = true;                 // single-machine testing
    bool useKernelspaceStamps = false;    // use timestamps taken in kernel space
};

struct SyncPair {                       // from the most recent Sync
    std::chrono::nanoseconds t1{};      // master's send time
    std::chrono::nanoseconds t2{};      // our arrival time
    bool valid = false;
};

struct PendingRequest {                 // the DelayReq we are waiting on
    std::chrono::nanoseconds t3{};      // our send time
    std::uint32_t seq = 0;
    bool valid = false;
};

class ClockSync {
public:
    ClockSync(Config config, Role role);           // Role::Master or Role::Slave
    ~ClockSync();                                  // stops and joins the worker
    void start();                                  // spawns its own thread
    void stop();

    // Per-sample log hook, in place of the library writing to stdout. Set it
    // before start(); it is then read from the io thread without a lock.
    void onSample(std::function<void(const Sample&)> callback);

    [[nodiscard]] std::optional<ClockMapping> mapping() const noexcept;  // audio-thread safe
    [[nodiscard]] State state() const noexcept;      // Unsynced / Acquiring / Locked / Holdover
    [[nodiscard]] Stats stats() const noexcept;      // offset, path delay, estimate quality

private:
    asio::io_context io_;
    asio::ip::udp::socket socket_;
    asio::steady_timer timer_;
    TimestampedReceiver receiver_;
    std::thread worker_;

    // Set inside the lambda stop() posts, so it is only ever touched on the io
    // thread. operation_aborted is not enough on its own: a completion already
    // queued with no error when stop() runs would re-arm afterwards, and that
    // work is never cancelled, so io_.run() never returns and join() hangs.
    bool stopping_ = false;

    Config config_;
    Role role_;

    std::chrono::steady_clock::time_point deadline_;
    std::uint32_t sequence_;

    std::atomic<std::size_t> seq_;
    std::atomic<double> localRef_, masterRef_, skew_;

    asio::ip::udp::endpoint remote_;
    std::array<std::uint8_t, 64> buffer_;

    SyncPair lastSync_;
    PendingRequest pending_;

    std::atomic<std::uint64_t> unmatched_, noSync_, staleSync_;

    std::mt19937 rng_;

    // TX self-probe. With loopback on, every node also receives its own
    // datagrams; the nodeId filter drops them. Timing them costs nothing and
    // gives (transmit path + loopback): with kernel RX stamps that is close to
    // this node's transmit latency, with userspace stamps its whole local
    // stack. Either way two nodes can be compared.
    struct SentRecord { std::uint32_t seq = 0; std::chrono::nanoseconds t{}; bool valid = false; };
    static constexpr std::size_t kSentHistory = 16;
    std::array<SentRecord, kSentHistory> sent_{};
    std::size_t sentNext_ = 0;

    void recordSent(std::uint32_t seq, std::chrono::nanoseconds t) noexcept;
    [[nodiscard]] std::optional<std::chrono::nanoseconds> sentAt(std::uint32_t seq) const noexcept;
    void probeOwnPacket(const SyncMessage& msg, Clock::time_point arrival);

    std::atomic<std::uint64_t> txProbeCount_{0};
    std::atomic<double> txProbeFloor_{0.0};
    std::atomic<double> statFloorA_{0.0};
    std::atomic<double> statFloorB_{0.0};

    // Diagnostics are produced on the io thread and read from whatever thread
    // calls stats(); mirror them into atomics rather than racing on the Servo
    // and TimestampedReceiver members directly.
    std::atomic<double> statPathDelay_{0.0};
    std::atomic<double> statGate_{0.0};
    std::atomic<bool> statGateSeeded_{false};
    std::atomic<double> statKernelLag_{0.0};
    std::atomic<std::uint64_t> statRejected_{0};
    std::atomic<std::uint64_t> statTooSoon_{0};
    std::atomic<bool> statKernelStamps_{false};
    std::optional<StampMode> reportedMode_;   // io thread only
    std::atomic<std::uint64_t> userStamps_{0};

    Servo servo_;
    std::function<void(const Sample&)> onSample_;

    std::uint8_t syncMessageBuffer_[kMessageBytes];
    std::uint8_t delayReqMessageBuffer_[kMessageBytes];
    std::uint8_t delayRespMessageBuffer_[kMessageBytes];

    void armReceive();
    void handleReceive(std::size_t n, Clock::time_point t);

    void armSyncTimer();
    void sendSync();

    void armDelayReqTimer();
    void sendDelayReq();

    void updateMapping(double localRef, double masterRef, double skew);
    void publishStats();          // io thread -> the atomics above
    void reportStampMode();       // one-shot, once probation has settled
};

}

#endif  // CLOCK_SYNC_H