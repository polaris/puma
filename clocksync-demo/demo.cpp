#include "clock_sync.h"
#include "netint.h"
#include "spsc_ring.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <CLI/CLI.hpp>
#include <asio.hpp>

namespace {

SpscRing<clocksync::Sample, 64> ring;

// Written by the io thread when the ring is full, read by the main thread for
// the status line. Without it a starved drain would thin out the log with no
// trace of it in the file the analysis reads.
std::atomic<std::uint64_t> dropped{0};

// Runs on the io thread, so it only hands the sample over; the formatting and
// the write happen on the main thread in drainSamples().
void logSample(const clocksync::Sample& s) {
    if (!ring.push(s)) dropped.fetch_add(1, std::memory_order_relaxed);
}

// The per-sample log goes to stdout, the human status line to stderr, so that
// `clocksync-demo -r slave > run.log` keeps the log machine-readable and still
// shows progress on the terminal.
void drainSamples() {
    clocksync::Sample s{};
    while (ring.pop(s)) {
        std::cout << s.offset * 1e6 << " "        // theta_raw, us
                  << s.delay * 1e6 << " "         // delay, us
                  << s.mappedOffset * 1e6 << " "  // theta from mapping, us
                  << s.skew * 1e6 << " "          // ppm
                  << s.rejected << " "            // increments on reject
                  << s.tooSoon << "\n";
    }
}

// state() and quality are deliberately absent: ClockSync::state() is still a
// stub that returns State{}, so printing them would read "unsynced, quality 0"
// on a perfectly healthy link.
// The gate is empty until the servo has enough delay samples; a master never
// feeds the servo, so for it this stays "-" for the whole run.
std::string formatGate(const std::optional<double>& gate) {
    if (!gate) return "-";
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << *gate * 1e6 << "us";
    return out.str();
}

void printStatus(const clocksync::Stats& s) {
    std::cerr << "status"
              << "  offset=" << s.offset * 1e6 << "us"
              << "  path=" << s.pathDelay * 1e6 << "us"
              << "  gate=" << formatGate(s.gateThreshold)
              << "  stamps=" << (s.kernelTimestamps ? "kernel" : "user")
              << "  lag=" << s.kernelLag * 1e6 << "us"
              << "  rejected=" << s.rejected
              << "  tooSoon=" << s.tooSoon
              << "  unmatched=" << s.unmatched
              << "  noSync=" << s.noSync
              << "  staleSync=" << s.staleSync
              << "  dropped=" << dropped.load(std::memory_order_relaxed)
              << "\n";
}

}  // namespace

constexpr std::string_view kDefaultMulticastGroup = "239.255.0.2";
constexpr int kDefaultPort = 12346;
constexpr clocksync::Role kDefaultRole = clocksync::Role::Master;
constexpr int kDefaultSyncInterval = 125;
constexpr int kDefaultDelayReqInterval = 125;
constexpr int kDefaultNodeId = 0;
constexpr int kDefaultDomain = 0;
constexpr double kDefaultAcquireBandwidth = 0.5;
constexpr double kDefaultLockBandwidth = 0.05;

constexpr auto kStatusInterval = std::chrono::seconds{5};
constexpr auto kSampleLogInterval = std::chrono::milliseconds{100};

int main(int argc, char** argv) {
    std::cout << std::fixed << std::showpoint;
    std::cout << std::setprecision(5);
    std::cerr << std::fixed << std::setprecision(3);

    CLI::App app{"Clock sync demo"};
    argv = app.ensure_utf8(argv);  // proper Unicode handling on Windows

    std::string ifName;
    std::string group{kDefaultMulticastGroup};
    int port = kDefaultPort;
    int domain = kDefaultDomain;
    std::uint64_t nodeId = kDefaultNodeId;
    bool loopback = true;
    clocksync::Role role{kDefaultRole};
    int syncInterval = kDefaultSyncInterval;
    int delayReqInterval = kDefaultDelayReqInterval;
    double acquireBandwidth = kDefaultAcquireBandwidth;
    double lockBandwidth = kDefaultLockBandwidth;

    app.add_option("-i,--interface", ifName, "Network interface");
    app.add_option("-n,--nodeid", nodeId, "Node ID")
        ->required()
        ->check(CLI::NonNegativeNumber);
    std::map<std::string, clocksync::Role> roles{{"master", clocksync::Role::Master}, {"slave", clocksync::Role::Slave}};
    app.add_option("-r,--role", role, "Role (master or slave)")
        ->required()
        ->transform(CLI::CheckedTransformer(roles, CLI::ignore_case));
    app.add_option("-d,--domain", domain, "Domain")
        ->check(CLI::Range(0, 255))
        ->default_val(kDefaultDomain);
    app.add_option("-g,--group", group, "Multicast group")
        ->check(CLI::ValidIPV4)
        ->default_str(std::string{kDefaultMulticastGroup});
    app.add_option("-p,--port", port, "Multicast port")
        ->check(CLI::Range(1, 65535))
        ->default_val(kDefaultPort);
    app.add_option("-l,--loopback", loopback, "Loopback")
        ->default_val(true);
    app.add_option("--syncInterval", syncInterval, "Sync interval")
        ->check(CLI::Range(50, 500))
        ->default_val(kDefaultSyncInterval);
    app.add_option("--delayReqInterval", delayReqInterval, "DelayReq interval")
        ->check(CLI::Range(50, 500))
        ->default_val(kDefaultDelayReqInterval);
    app.add_option("--acquireBandwidth", acquireBandwidth, "Acquire bandwidth")
        ->check(CLI::PositiveNumber)
        ->default_val(kDefaultAcquireBandwidth);
    app.add_option("--lockBandwidth", lockBandwidth, "Lock bandwidth")
        ->check(CLI::PositiveNumber)
        ->default_val(kDefaultLockBandwidth);
    CLI11_PARSE(app, argc, argv);  // handles --help, errors, exit codes

    // Everything below can throw: enumerate() and the ClockSync constructor,
    // which binds the socket and joins the multicast group, both fail with an
    // asio::system_error on a busy port or an interface that cannot join.
    try {
        const auto all = net::enumerate();
        std::cout << "interfaces:\n";
        for (const auto& i : all) {
            std::cout << "  " << i.name << "  " << i.address.to_string()
                      << "  idx=" << i.index;
            if (!i.description.empty()) std::cout << "  (" << i.description << ")";
            std::cout << "\n";
        }
        const auto chosen = ifName.empty() ? net::selectDefault() : net::find(ifName);
        if (!chosen) {
            std::cerr << (ifName.empty() ? "ambiguous or none; name one explicitly\n"
                                         : "no such interface\n");
            return 1;
        }
        std::cout << "using " << chosen->name << " " << chosen->address.to_string() << "\n";

        clocksync::Config config{
            .group = asio::ip::udp::endpoint(asio::ip::make_address(group), port),
            .iface = *chosen,
            .nodeId = nodeId,
            .domain = static_cast<std::uint8_t>(domain),
            .syncInterval = std::chrono::milliseconds{syncInterval},
            .delayReqInterval = std::chrono::milliseconds{delayReqInterval},
            .acquireBandwidth = acquireBandwidth,
            .lockBandwidth = lockBandwidth,
            .loopback = loopback
        };
        clocksync::ClockSync cs{config, role};
        cs.onSample(logSample);   // before start(): read from the io thread
        cs.start();

        asio::io_context wait;
        asio::signal_set signals{wait, SIGINT, SIGTERM};
        signals.async_wait([&](auto, int) {
            std::cerr << "\nshutting down\n";
            wait.stop();
        });

        asio::steady_timer sampleLog{wait, kSampleLogInterval};
        std::function<void(const asio::error_code&)> tack = [&](const asio::error_code& ec) {
            if (ec) return;
            drainSamples();
            sampleLog.expires_at(sampleLog.expiry() + kSampleLogInterval);
            sampleLog.async_wait(tack);
        };
        sampleLog.async_wait(tack);

        asio::steady_timer status{wait, kStatusInterval};
        std::function<void(const asio::error_code&)> tick = [&](const asio::error_code& ec) {
            if (ec) return;
            printStatus(cs.stats());
            status.expires_at(status.expiry() + kStatusInterval);
            status.async_wait(tick);
        };
        status.async_wait(tick);

        wait.run();

        // run() returns on the signal with up to a full ring still unwritten.
        // stop() first, so the io thread is joined and done pushing, then take
        // the tail; without this every Ctrl-C loses the last samples.
        cs.stop();
        drainSamples();
        if (const auto lost = dropped.load(std::memory_order_relaxed); lost > 0) {
            std::cerr << "dropped " << lost << " samples\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
