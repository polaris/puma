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
    net::Interface iface;
    std::uint8_t domain = 0;              // two systems, one LAN, no interference
    std::uint64_t nodeId = 0;             // used to ignore our own packets
    std::chrono::milliseconds syncInterval{125};        // 8/s
    std::chrono::milliseconds delayReqInterval{125};    // 8/s
    double acquireBandwidth = 0.5;        // Hz
    double lockBandwidth    = 0.05;       // Hz
    bool loopback = true;                 // single-machine testing
    // force the receiver to use timestamps taken in user space
    bool forceUserspaceStamps = false;
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

    void sendMessage(const SyncMessage& msg);

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