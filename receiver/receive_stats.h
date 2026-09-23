#ifndef RECEIVE_STATS_H
#define RECEIVE_STATS_H

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <asio/error_code.hpp>

// Written and read on the io thread only.
struct ReceiveStats {
    std::uint64_t discontinuities = 0;
    std::uint64_t ringDrops = 0;        // packets the ring had no room for
    std::uint64_t lostFrames = 0;       // frames the sender sent that never arrived
    std::uint64_t concealFailures = 0;  // holes the ring was too full to patch
    std::uint64_t resyncs = 0;          // holes too large to patch at all
    std::uint64_t sizeMismatches = 0;   // payloads that are not frames * bytesPerFrame
    std::uint64_t receiveErrors = 0;
    std::uint64_t runtPackets = 0;      // shorter than the header
    std::uint64_t frameCountChanges = 0;

    // Details of the most recent occurrence, for the status reporter.
    asio::error_code lastReceiveError;
    std::size_t lastMismatchBytes = 0;
    std::uint32_t lastMismatchFrames = 0;
};

// Written on the audio thread, read on the io thread. On cache lines of its
// own, so the audio thread's stores do not contend with ReceiveStats.
struct alignas(128) PlaybackStats {
    std::atomic<std::uint64_t> underrunFrames{0};
    std::atomic<double> deviceRate{0.0};    // frames per second of steady_clock time; 0 until known
};
static_assert(std::atomic<double>::is_always_lock_free);

// Figures for one reporting interval, gathered per packet.
struct ReceiveWindow {
    std::uint64_t packets = 0;
    std::size_t minFill = std::numeric_limits<std::size_t>::max();
    std::size_t maxFill = 0;

    std::uint64_t timingUpdates = 0;
    double sumSquaredError = 0.0;       // seconds²
    double maxAbsError = 0.0;           // seconds

    void addFill(std::size_t frames) noexcept {
        ++packets;
        minFill = std::min(minFill, frames);
        maxFill = std::max(maxFill, frames);
    }

    void addError(double error) noexcept {
        ++timingUpdates;
        sumSquaredError += error * error;
        maxAbsError = std::max(maxAbsError, std::abs(error));
    }
};

// What the status reporter takes from the packet receiver once per interval.
struct ReceiverSnapshot {
    ReceiveWindow window;
    double senderRate = 0.0;            // frames per second of steady_clock time; 0 until locked on
    std::uint32_t framesPerPacket = 0;  // 0 until the first packet
};

#endif  // RECEIVE_STATS_H
