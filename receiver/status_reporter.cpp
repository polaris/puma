#include "status_reporter.h"

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <utility>

#include <asio/post.hpp>

#include "terminal.h"

StatusReporter::StatusReporter(asio::io_context& control, asio::io_context& receiveIo, SnapshotSource takeSnapshot,
                               const PlaybackStats& playback, unsigned int nominalRate, std::size_t bytesPerFrame,
                               Clock::time_point origin)
: control_{control}
, receiveIo_{receiveIo}
, timer_{control}
, takeSnapshot_{std::move(takeSnapshot)}
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
        requestSnapshot();
        next_ += kInterval;
        schedule();
    });
}

void StatusReporter::requestSnapshot() {
    asio::post(receiveIo_, [this] {
        // On the receive thread: copy, hand back, done. Formatting and writing
        // happen on the control thread.
        asio::post(control_, [this, snapshot = takeSnapshot_()] {
            if (!stopping_) {
                report(snapshot);
            }
        });
    });
}

void StatusReporter::report(const ReceiverSnapshot& snapshot) {
    reportEvents(snapshot);
    showStatus(statusLine(snapshot));
}

void StatusReporter::reportEvents(const ReceiverSnapshot& snapshot) {
    const ReceiveStats& stats = snapshot.stats;
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

    if (stats.receiveErrors > seenReceiveErrors_) {
        std::ostringstream msg;
        msg << "receive: " << stats.lastReceiveError.message();
        if (stats.receiveErrors - seenReceiveErrors_ > 1) {
            msg << " (and " << (stats.receiveErrors - seenReceiveErrors_ - 1) << " more)";
        }
        printEvent(msg.str());
        seenReceiveErrors_ = stats.receiveErrors;
    }

    if (stats.sizeMismatches > 0 && !reportedSizeMismatch_) {
        std::ostringstream msg;
        msg << "payload is " << stats.lastMismatchBytes << " bytes for "
            << stats.lastMismatchFrames << " frames, but this device wants "
            << bytesPerFrame_ << " bytes per frame";
        printEvent(msg.str());
        reportedSizeMismatch_ = true;
    }
}

std::string StatusReporter::statusLine(const ReceiverSnapshot& snapshot) const {
    const ReceiveStats& stats = snapshot.stats;
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

    line << "  headroom ";
    if (const std::int64_t headroom = playback_.headroom.load(std::memory_order_relaxed);
        headroom != PlaybackStats::kHeadroomUnknown) {
        line << headroom << '/' << playback_.targetHeadroom.load(std::memory_order_relaxed);
    } else {
        line << "--";
    }

    line << "  ring ";
    if (window.packets > 0) {
        line << window.minFill << '-' << window.maxFill;
    } else {
        line << "--";
    }

    line << "  lost " << stats.lostFrames
         << "  drops " << stats.ringDrops
         << "  underruns " << playback_.underrunFrames.load(std::memory_order_relaxed)
         << "  trimmed " << playback_.trimmedFrames.load(std::memory_order_relaxed);

    // Rare trouble, shown once it has happened.
    appendIfAny(line, "conceal-fail", stats.concealFailures);
    appendIfAny(line, "resyncs", stats.resyncs);
    appendIfAny(line, "size-mismatch", stats.sizeMismatches);
    appendIfAny(line, "rx-err", stats.receiveErrors);
    appendIfAny(line, "runts", stats.runtPackets);

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
