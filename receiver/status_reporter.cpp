#include "status_reporter.h"

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <utility>

#include "terminal.h"

StatusReporter::StatusReporter(asio::io_context& io, SnapshotSource takeSnapshot, const ReceiveStats& stats,
                               const PlaybackStats& playback, unsigned int nominalRate, std::size_t bytesPerFrame,
                               Clock::time_point origin)
: timer_{io}
, takeSnapshot_{std::move(takeSnapshot)}
, stats_{stats}
, playback_{playback}
, nominalRate_{nominalRate}
, bytesPerFrame_{bytesPerFrame}
, origin_{origin}
, interactive_{term::isTerminal(stderr)} {
}

void StatusReporter::start() {
    next_ = Clock::now() + kInterval;
    schedule();
}

void StatusReporter::stop() {
    stopping_ = true;
    timer_.cancel();
    if (shownWidth_ > 0) {
        std::cerr << '\n';
        shownWidth_ = 0;
    }
}

void StatusReporter::schedule() {
    timer_.expires_at(next_);
    timer_.async_wait([this](const asio::error_code& ec) {
        if (stopping_ || ec) {
            return;
        }
        report();
        next_ += kInterval;
        schedule();
    });
}

void StatusReporter::report() {
    const ReceiverSnapshot snapshot = takeSnapshot_();
    reportEvents(snapshot);
    showStatus(statusLine(snapshot));
}

void StatusReporter::reportEvents(const ReceiverSnapshot& snapshot) {
    const std::uint32_t frames = snapshot.framesPerPacket;
    if (frames != seenFramesPerPacket_) {
        std::ostringstream msg;
        if (seenFramesPerPacket_ == 0) {
            msg << "receiving " << frames << " frames per packet at nominal " << nominalRate_ << " Hz";
        } else {
            msg << "frames per packet changed " << seenFramesPerPacket_ << " -> " << frames << ", resyncing";
        }
        printEvent(msg.str());
        seenFramesPerPacket_ = frames;
    }

    if (stats_.receiveErrors > seenReceiveErrors_) {
        std::ostringstream msg;
        msg << "receive: " << stats_.lastReceiveError.message();
        if (stats_.receiveErrors - seenReceiveErrors_ > 1) {
            msg << " (and " << (stats_.receiveErrors - seenReceiveErrors_ - 1) << " more)";
        }
        printEvent(msg.str());
        seenReceiveErrors_ = stats_.receiveErrors;
    }

    if (stats_.sizeMismatches > 0 && !reportedSizeMismatch_) {
        std::ostringstream msg;
        msg << "payload is " << stats_.lastMismatchBytes << " bytes for "
            << stats_.lastMismatchFrames << " frames, but this device wants "
            << bytesPerFrame_ << " bytes per frame";
        printEvent(msg.str());
        reportedSizeMismatch_ = true;
    }
}

std::string StatusReporter::statusLine(const ReceiverSnapshot& snapshot) const {
    const ReceiveWindow& window = snapshot.window;
    const double senderRate = snapshot.senderRate;
    const double deviceRate = playback_.deviceRate.load(std::memory_order_relaxed);
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - origin_);

    std::ostringstream line;
    line << std::fixed;
    line << '[' << elapsed.count() << "s]";

    line << "  drift ";
    if (senderRate > 0.0 && deviceRate > 0.0) {
        appendPpm(line, senderRate / deviceRate);
    } else {
        line << "--";
    }

    line << "  jitter ";
    if (window.timingUpdates > 0) {
        const double rms = std::sqrt(window.sumSquaredError / static_cast<double>(window.timingUpdates));
        line << std::setprecision(0) << rms * 1e6 << '/' << window.maxAbsError * 1e6 << " us";
    } else {
        line << "--";
    }

    line << "  ring ";
    if (window.packets > 0) {
        line << window.minFill << '-' << window.maxFill;
    } else {
        line << "--";
    }

    line << "  lost " << stats_.lostFrames
         << "  drops " << stats_.ringDrops
         << "  underruns " << playback_.underrunFrames.load(std::memory_order_relaxed);

    // Rare trouble, shown once it has happened.
    appendIfAny(line, "conceal-fail", stats_.concealFailures);
    appendIfAny(line, "resyncs", stats_.resyncs);
    appendIfAny(line, "size-mismatch", stats_.sizeMismatches);
    appendIfAny(line, "rx-err", stats_.receiveErrors);
    appendIfAny(line, "runts", stats_.runtPackets);

    // Last, so a narrow terminal cuts the details rather than the above.
    line << "  sender ";
    appendRate(line, senderRate);
    line << "  device ";
    appendRate(line, deviceRate);

    return line.str();
}

void StatusReporter::appendRate(std::ostringstream& line, double rate) const {
    if (rate <= 0.0) {
        line << "--";
        return;
    }
    line << std::setprecision(2) << rate << " Hz (";
    appendPpm(line, rate / nominalRate_);
    line << ')';
}

void StatusReporter::appendPpm(std::ostringstream& line, double ratio) {
    line << std::showpos << std::setprecision(1) << (ratio - 1.0) * 1e6 << std::noshowpos << " ppm";
}

void StatusReporter::appendIfAny(std::ostringstream& line, const char* label, std::uint64_t count) {
    if (count > 0) {
        line << "  " << label << ' ' << count;
    }
}

// A line of its own, above the status line.
void StatusReporter::printEvent(const std::string& msg) {
    if (shownWidth_ > 0) {
        std::cerr << '\r' << std::string(shownWidth_, ' ') << '\r';
        shownWidth_ = 0;
    }
    std::cerr << msg << '\n';
}

// Redraws the status line in place on a terminal, appends a line otherwise.
void StatusReporter::showStatus(std::string line) {
    if (!interactive_) {
        std::cerr << line << '\n';
        return;
    }
    // A line that wraps cannot be redrawn with '\r'.
    const std::size_t columns = term::stderrColumns();
    if (columns > 1 && line.size() >= columns) {
        line.resize(columns - 1);
    }
    const std::size_t width = line.size();
    if (width < shownWidth_) {
        line.append(shownWidth_ - width, ' ');
    }
    std::cerr << '\r' << line << std::flush;
    shownWidth_ = width;
}
