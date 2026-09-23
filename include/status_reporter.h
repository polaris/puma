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
// counts as lines of their own above it.
//
// Runs on a control io_context of its own, never on the receive thread: a
// terminal that stalls a write must not hold up packets. Once per interval it
// posts a request to the receive thread, which makes a snapshot and posts it
// back by value, so neither side locks and ReceiveStats stays single-threaded.
class StatusReporter {
public:
    using Clock = std::chrono::steady_clock;
    using SnapshotSource = std::function<ReceiverSnapshot()>;

    static constexpr auto kInterval = std::chrono::seconds(1);

    // `takeSnapshot` is called on the thread running `receiveIo`, and is
    // expected to start a new ReceiveWindow each time.
    StatusReporter(asio::io_context& control, asio::io_context& receiveIo, SnapshotSource takeSnapshot,
                   const PlaybackStats& playback, unsigned int nominalRate, std::size_t bytesPerFrame,
                   Clock::time_point origin);

    StatusReporter(const StatusReporter&) = delete;
    StatusReporter& operator=(const StatusReporter&) = delete;

    void start();

    // On the control thread, or after `control` has stopped running. Leaves
    // the last status line on screen; snapshots still in flight are dropped.
    void stop();

private:
    void schedule();
    void requestSnapshot();
    void report(const ReceiverSnapshot& snapshot);
    void reportEvents(const ReceiverSnapshot& snapshot);
    [[nodiscard]] std::string statusLine(const ReceiverSnapshot& snapshot) const;

    void appendRate(std::ostringstream& line, double rate) const;
    static void appendPpm(std::ostringstream& line, double ratio);
    static void appendIfAny(std::ostringstream& line, const char* label, std::uint64_t count);

    void printEvent(const std::string& msg);
    void showStatus(std::string line);

    asio::io_context& control_;
    asio::io_context& receiveIo_;
    asio::steady_timer timer_;
    Clock::time_point next_;
    const SnapshotSource takeSnapshot_;
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
