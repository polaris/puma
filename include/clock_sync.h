#ifndef CLOCK_SYNC_H
#define CLOCK_SYNC_H

#include "netint.h"
#include "sync_message.h"

#include <asio.hpp>
#include <chrono>
#include <optional>

namespace clocksync {

using Clock = std::chrono::steady_clock;

enum class Role { Master, Slave };

struct ClockMapping {
    double localRef;     // a local steady_clock instant, seconds
    double masterRef;    // the master time at that instant
    double skew;         // master seconds per local second, minus 1
};

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
    std::chrono::milliseconds delayReqInterval{500};    // randomized per node
    double acquireBandwidth = 1.0;        // Hz
    double lockBandwidth    = 0.05;       // Hz
    bool loopback = true;                 // single-machine testing
};

struct State {
    enum Value { Unsynced, Acquiring, Locked, Holdover } value = Unsynced;
    double holdoverAge = 0;  // seconds since last update
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

    std::size_t delayReqSeq_;

    std::chrono::nanoseconds t1_, t2_, t3_;

    void sendMessage(const SyncMessage& msg);

    void armReceive();
    void handleReceive(std::size_t n, Clock::time_point t);

    void armSyncTimer();
    void sendSync();

    void armDelayReqTimer();
    void sendDelayReq();

    void updateMapping(double localRef, double masterRef, double skew);
};

[[nodiscard]] double localToMaster(double local, const ClockMapping& mapping) noexcept;
[[nodiscard]] double masterToLocal(double master, const ClockMapping& mapping) noexcept;

}

#endif  // CLOCK_SYNC_H