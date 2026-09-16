#ifndef CLOCK_SYNC_H
#define CLOCK_SYNC_H

#include "netint.h"
#include "sync_message.h"
#include "clock_helper.h"
#include "servo.h"

#include <asio.hpp>
#include <chrono>
#include <optional>

namespace clocksync {

enum class Role { Master, Slave };

struct Stats {
    double offset;       // master - local, seconds
    double pathDelay;    // seconds
    double quality;      // 0..1, 1 is best
};

struct Config {
    asio::ip::udp::endpoint group;        // 239.255.0.2:12346 — not the audio group
    net::Interface iface;
    std::uint8_t domain = 0;              // two systems, one LAN, no interference
    std::uint64_t nodeId = 0;             // 0 = generate; used to ignore our own packets
    std::chrono::milliseconds syncInterval{125};        // 8/s
    std::chrono::milliseconds delayReqInterval{125};    // randomized per node
    double acquireBandwidth = 0.5;        // Hz
    double lockBandwidth    = 0.05;       // Hz
    bool loopback = true;                 // single-machine testing
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

    [[nodiscard]] std::optional<ClockMapping> mapping() const noexcept;  // audio-thread safe
    [[nodiscard]] State state() const noexcept;      // Unsynced / Acquiring / Locked / Holdover
    [[nodiscard]] Stats stats() const noexcept;      // offset, path delay, estimate quality

private:
    asio::io_context io_;
    asio::ip::udp::socket socket_;
    asio::steady_timer timer_;
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

    Servo servo_;

    void sendMessage(const SyncMessage& msg);

    void armReceive();
    void handleReceive(std::size_t n, Clock::time_point t);

    void armSyncTimer();
    void sendSync();

    void armDelayReqTimer();
    void sendDelayReq();

    void updateMapping(double localRef, double masterRef, double skew);
};

}

#endif  // CLOCK_SYNC_H