#ifndef STATUS_REPORTER_H
#define STATUS_REPORTER_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <sstream>
#include <string>

#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>

#include "receive_stats.h"

// Keeps a status line up to date on stderr, and prints what the receiver only
// counts as lines of their own above it. Runs on the io thread, like the
// packet receiver, so it reads ReceiveStats without synchronisation.
class StatusReporter {
public:
    using Clock = std::chrono::steady_clock;
    using SnapshotSource = std::function<ReceiverSnapshot()>;

    static constexpr auto kInterval = std::chrono::seconds(1);

    // `takeSnapshot` is called once per interval, on the io thread, and is
    // expected to start a new ReceiveWindow each time.
    StatusReporter(asio::io_context& io, SnapshotSource takeSnapshot, const ReceiveStats& stats,
                   const PlaybackStats& playback, unsigned int nominalRate, std::size_t bytesPerFrame,
                   Clock::time_point origin);

    StatusReporter(const StatusReporter&) = delete;
    StatusReporter& operator=(const StatusReporter&) = delete;

    void start();

    // Io thread only. Leaves the last status line on screen. No final report:
    // playback has stopped by now, so the ring fill would be misleading.
    void stop();

private:
    void schedule();
    void report();
    void reportEvents(const ReceiverSnapshot& snapshot);
    [[nodiscard]] std::string statusLine(const ReceiverSnapshot& snapshot) const;

    void appendRate(std::ostringstream& line, double rate) const;
    static void appendPpm(std::ostringstream& line, double ratio);
    static void appendIfAny(std::ostringstream& line, const char* label, std::uint64_t count);

    void printEvent(const std::string& msg);
    void showStatus(std::string line);

    asio::steady_timer timer_;
    const SnapshotSource takeSnapshot_;
    const ReceiveStats& stats_;
    const PlaybackStats& playback_;
    const unsigned int nominalRate_;
    const std::size_t bytesPerFrame_;
    const Clock::time_point origin_;
    const bool interactive_;

    bool stopping_ = false;
    std::size_t shownWidth_ = 0;          // characters of status line on screen

    std::uint32_t seenFramesPerPacket_ = 0;
    std::uint64_t seenReceiveErrors_ = 0;
    bool reportedSizeMismatch_ = false;
};

#endif  // STATUS_REPORTER_H
