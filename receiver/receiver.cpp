#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <asio.hpp>
#include <CLI/CLI.hpp>

// The audio headers below pull in the miniaudio declarations; the
// implementation block sits outside miniaudio.h's include guard, so it has to
// be compiled here, once.
#define MA_IMPLEMENTATION
#include <miniaudio.h>

#include "audio_context.h"
#include "audio_packet.h"
#include "audio_player.h"
#include "clock_sync.h"
#include "exit_codes.h"
#include "frame_ring.h"
#include "netint.h"
#include "presentation_timeline.h"
#include "rate_loop.h"
#include "receive_stats.h"
#include "resampler.h"
#include "shutdown_signal.h"
#include "spsc_ring.h"
#include "status_reporter.h"
#include "stream_defaults.h"
#include "thread_priority.h"
#include "time_filter.h"

using asio::ip::udp;
using Clock = std::chrono::steady_clock;

using streaming::kAudioPacketHeaderBytes;
using streaming::kBandwidth;
using streaming::kClockGroup;
using streaming::kClockPort;
using streaming::kNumPeriods;
using streaming::kPeriodSizeInFrames;
using streaming::kStartBandwidth;
using streaming::kStartSeconds;

namespace {

constexpr unsigned int kDefaultSampleRate = 48000;

// Frames wait here from arrival until their presentation time, about the
// sender's latency L: ~170 ms, enough for L up to about 150 ms.
constexpr std::size_t kRingFrames = 8192;
using Ring = FrameRing<kRingFrames>;

// Network thread -> audio thread: one (t, k) point per packet for the timeline.
using ReportQueue = SpscRing<NetReport, 64>;

// From the start of a callback to its sound leaving the speaker. Differs per
// device, driver and buffer setting, and miniaudio does not report it
// reliably: measure it, and pass it per receiver.
constexpr double kDefaultOutputLatencyMs = 0.0;

// Further off than this, the receiver re-aligns instead of steering back.
constexpr double kRealignMs = 5.0;

// CPU the receive thread needs per packet, at most; for the real-time scheduler.
constexpr auto kReceiveComputation = std::chrono::microseconds(500);

struct PlaybackState {
    unsigned int deviceRate = 0;
    std::size_t bytesPerFrame = 0;
    std::size_t headroomWindowFrames = 0;   // how long to watch the lowest headroom before reporting it
    double outputLatency = 0.0;         // seconds, callback start to speaker

    Ring* ring = nullptr;
    ReportQueue* reports = nullptr;
    PlaybackStats* stats = nullptr;
    const clocksync::ClockSync* clock = nullptr;

    Clock::time_point origin;
    double originSeconds = 0.0;         // origin as clocksync's local time: seconds since the steady epoch

    // Audio thread only.
    TimeFilter timeFilter{};
    PresentationTimeline timeline{};
    std::optional<ClockMapping> mapping;    // the last one read
    RateLoop loop{};
    Resampler resampler{};
    std::int64_t windowMinHeadroom = std::numeric_limits<std::int64_t>::max();
    std::size_t windowFrames = 0;
};

void printReceiveStats(const ReceiveStats& stats, const PlaybackStats& playback) {
    std::cerr << "discontinuities " << stats.discontinuities
              << ", lost frames " << stats.lostFrames
              << ", conceal failures " << stats.concealFailures
              << ", resyncs " << stats.resyncs
              << ", ring drops " << stats.ringDrops
              << ", size mismatches " << stats.sizeMismatches
              << ", receive errors " << stats.receiveErrors
              << ", runt packets " << stats.runtPackets
              << ", bad headers " << stats.badHeaders
              << ", session changes " << stats.sessionChanges
              << ", frame count changes " << stats.frameCountChanges
              << ", underrun frames " << playback.underrunFrames.load()
              << ", trimmed frames " << playback.trimmedFrames.load()
              << ", late frames " << playback.lateFrames.load()
              << ", realigns " << playback.realigns.load()
              << ", clock steps " << playback.clockSteps.load()
              << "\n";
}

[[nodiscard]] double trackDeviceClock(PlaybackState& s, ma_uint32 frameCount) {
    const auto t = std::chrono::duration<double>(Clock::now() - s.origin).count();
    // Also true on the first call. miniaudio does not promise a fixed callback
    // size, and the filter assumes one, so it starts over when the size changes.
    if (frameCount != s.timeFilter.framesPerPeriod()) {
        s.timeFilter.configure(kBandwidth, frameCount, s.deviceRate, kStartBandwidth, kStartSeconds);
        s.timeFilter.reset(t);
        s.loop.setFramesPerCallback(frameCount);
    } else {
        s.timeFilter.update(t);
        s.stats->deviceRate.store(s.timeFilter.rate(), std::memory_order_relaxed);
    }
    return s.timeFilter.time();
}

// The ring ran dry: start over. The loop waits until the due frame is in the
// ring again, which also covers a sender that stopped.
void onUnderrun(PlaybackState& s, ma_uint32 frameCount) {
    s.loop.restart();
    s.resampler.reset();
    s.stats->underrunFrames.fetch_add(frameCount, std::memory_order_relaxed);
}

// The rate loop's error (see rate_loop.h): how far the frame due at the
// speaker when this callback is heard lies ahead of the next frame played,
// in sender frames. Nothing until there is a clock mapping and a timeline.
[[nodiscard]] std::optional<double> timingError(PlaybackState& s, double tD) {
    // The seqlock read can fail while the clock-sync thread writes; the last
    // mapping is then a fraction of a second old, which is fine.
    if (const auto mapping = s.clock->mapping()) {
        if (s.mapping && mapping->generation != s.mapping->generation) {
            // A fresh estimate may have jumped: re-align to it.
            s.loop.restart();
            s.resampler.reset();
            s.stats->clockSteps.fetch_add(1, std::memory_order_relaxed);
        }
        s.mapping = mapping;
    }
    if (!s.mapping) {
        return std::nullopt;
    }

    const double heard = localToMaster(s.originSeconds + tD + s.outputLatency, *s.mapping);
    const auto ahead = s.timeline.framesAhead(heard, s.ring->readPosition());
    if (!ahead) {
        return std::nullopt;
    }
    return *ahead + s.resampler.inputDistance();
}

// Lowest headroom (ring fill beyond what a read takes) over a window, for the status line.
void trackHeadroom(PlaybackState& s, std::int64_t headroom, ma_uint32 frameCount) {
    s.windowMinHeadroom = std::min(s.windowMinHeadroom, headroom);
    s.windowFrames += frameCount;
    if (s.windowFrames >= s.headroomWindowFrames) {
        s.stats->headroom.store(s.windowMinHeadroom, std::memory_order_relaxed);
        s.windowMinHeadroom = std::numeric_limits<std::int64_t>::max();
        s.windowFrames = 0;
    }
}

void publishLoop(const PlaybackState& s) {
    s.stats->loopPhase.store(static_cast<int>(s.loop.phase()), std::memory_order_relaxed);
    s.stats->correction.store(s.loop.correction(), std::memory_order_relaxed);
    s.stats->delayError.store(s.loop.error(), std::memory_order_relaxed);
    s.stats->realigns.store(s.loop.realigns(), std::memory_order_relaxed);
}

void playSilence(const PlaybackState& s, void* output, ma_uint32 frameCount) {
    std::memset(output, 0, frameCount * s.bytesPerFrame);
}

// Every callback measures how far off its sound will be heard, steers the
// resampling ratio from that (see rate_loop.h), and then reads exactly what
// the resampler needs for this callback.
void onPlayback(void* user, void* output, ma_uint32 frameCount) {
    auto& s = *static_cast<PlaybackState*>(user);
    const double t = trackDeviceClock(s, frameCount);

    for (NetReport report; s.reports->pop(report); ) {
        if (report.restart) {
            s.loop.restart();       // a new stream, a new timeline: re-align
        }
        s.timeline.add(report);
    }

    const auto error = timingError(s, t);
    const auto step = s.loop.update(t, error, s.ring->availableRead(), s.resampler.inputFor(frameCount));
    publishLoop(s);

    if (step.trim > 0) {
        // Only this thread reads, so the fill can only have grown since.
        (void)s.ring->discard(step.trim);
        (step.late ? s.stats->lateFrames : s.stats->trimmedFrames).fetch_add(step.trim, std::memory_order_relaxed);
    }

    if (!step.play) {
        s.resampler.reset();
        s.stats->headroom.store(PlaybackStats::kHeadroomUnknown, std::memory_order_relaxed);
        playSilence(s, output, frameCount);
        return;
    }

    s.resampler.setRatio(step.ratio);
    const std::size_t need = s.resampler.inputFor(frameCount);
    const std::size_t fill = s.ring->availableRead();
    const std::int64_t headroom = static_cast<std::int64_t>(fill) - static_cast<std::int64_t>(need);
    trackHeadroom(s, headroom, frameCount);
    if (need > fill) {
        onUnderrun(s, frameCount);
        playSilence(s, output, frameCount);
        return;
    }

    // Resampled in place: the producer cannot reuse the slots before the commit.
    const Regions in = s.ring->acquireRead(need);
    s.resampler.process(reinterpret_cast<const std::int16_t*>(in.region1().buf), in.region1().len,
                        reinterpret_cast<const std::int16_t*>(in.region2().buf), in.region2().len,
                        static_cast<std::int16_t*>(output), frameCount);
    (void)s.ring->commitRead(need);
}

class PacketReceiver {
public:
    PacketReceiver(udp::socket& rx, Ring& ring, ReportQueue& reports, ReceiveStats& stats,
                   const clocksync::ClockSync& clock,
                   std::size_t bytesPerFrame, unsigned int sampleRate, Clock::time_point origin)
    : rx_{rx}
    , ring_{ring}
    , reports_{reports}
    , stats_{stats}
    , clock_{clock}
    , bytesPerFrame_{bytesPerFrame}
    , sampleRate_{sampleRate}
    , origin_{origin} {
    }

    PacketReceiver(const PacketReceiver&) = delete;
    PacketReceiver& operator=(const PacketReceiver&) = delete;

    void start() {
        arm();
    }

    void stop() {
        stopping_ = true;
        asio::error_code ignored;
        rx_.close(ignored);
    }

    // Hands over the current window, starting a new one, with the filter's state.
    [[nodiscard]] ReceiverSnapshot takeSnapshot() noexcept {
        return {
            .stats = stats_,
            .window = std::exchange(window_, {}),
            .senderRate = filter_.ready() ? filter_.rate() : 0.0,
            .framesPerPacket = filter_.framesPerPeriod(),
        };
    }

private:
    void arm() {
        rx_.async_receive(asio::buffer(buf_),
            [this](const asio::error_code& ec, std::size_t n) { onReceive(ec, n); });
    }

    void onReceive(const asio::error_code& ec, std::size_t n) {
        if (stopping_ || ec == asio::error::operation_aborted) {
            return;
        }
        if (ec) {
            arm();
            ++stats_.receiveErrors;
            stats_.lastReceiveError = ec;
            return;
        }
        if (n < kAudioPacketHeaderBytes) {
            arm();
            ++stats_.runtPackets;
            return;
        }

        const auto arrival = Clock::now();
        const auto decoded = streaming::decodeAudioPacketHeader(buf_.data(), n);
        if (!decoded) {
            arm();
            ++stats_.badHeaders;
            return;
        }
        const streaming::AudioPacketHeader& header = *decoded;

        if (warmupPackets_ > 0) {
            --warmupPackets_;
            session_ = header.session;
            expectedSeq_ = header.seq + 1;
            arm();
            return;
        }

        // A restarted sender counts from a sequence of its own: pick it up
        // as a new stream rather than as a gap.
        if (header.session != session_) {
            ++stats_.sessionChanges;
            session_ = header.session;
            expectedSeq_ = header.seq;
            lostTrack();
        }

        const std::uint32_t gap = header.seq - expectedSeq_;
        if (gap != 0) {
            concealGap(gap, header.frames);
        }
        expectedSeq_ = header.seq + 1;

        // Where the packet's first frame lands, past any concealed gap.
        const std::uint64_t first = ring_.writePosition();
        const bool stored = storePayload(header.frames, n - kAudioPacketHeaderBytes);

        arm();

        window_.addFill(ring_.availableRead());
        trackTiming(arrival, header.frames);
        trackLead(arrival, header.t);
        if (stored) {
            report(header.t, first);
        } else {
            // The positions after a missing packet no longer match the
            // stamps: the timeline has to start over.
            restartPending_ = true;
        }
    }

    // The timing starts over, and with it the presentation timeline.
    void lostTrack() {
        filter_.invalidate();
        restartPending_ = true;
    }

    // A point on the presentation timeline (see presentation_timeline.h): the
    // frame at ring position `first` is to be heard at master time `stamp`.
    void report(std::uint64_t stamp, std::uint64_t first) {
        if (reports_.push({static_cast<double>(stamp) * 1e-9, first, restartPending_})) {
            restartPending_ = false;
        }
    }

    // How long before its presentation time the packet arrived, in master
    // time: the safety margin L leaves this receiver.
    void trackLead(Clock::time_point arrival, std::uint64_t stamp) {
        const auto mapping = clock_.mapping();
        if (!mapping) {
            return;
        }
        const double arrived = localToMaster(toSeconds(sinceEpoch(arrival)), *mapping);
        window_.addLead(static_cast<double>(stamp) * 1e-9 - arrived);
    }

    void concealGap(std::uint32_t gap, std::uint32_t frames) {
        ++stats_.discontinuities;
        const std::size_t missing = static_cast<std::size_t>(gap) * frames;

        if (missing > kRingFrames) {
            ++stats_.resyncs;
            lostTrack();
            return;
        }

        stats_.lostFrames += missing;
        filter_.skip(gap);
        if (!ring_.writeSilence(missing)) {
            ++stats_.concealFailures;
        }
    }

    [[nodiscard]] bool storePayload(std::uint32_t frames, std::size_t payloadBytes) {
        if (payloadBytes != frames * bytesPerFrame_) {
            ++stats_.sizeMismatches;
            stats_.lastMismatchBytes = payloadBytes;
            stats_.lastMismatchFrames = frames;
            return false;
        }
        if (!ring_.write(buf_.data() + kAudioPacketHeaderBytes, frames)) {
            ++stats_.ringDrops;                // whole packet or nothing
            return false;
        }
        return true;
    }

    void trackTiming(Clock::time_point arrival, std::uint32_t frames) {
        const double t = std::chrono::duration<double>(arrival - origin_).count();

        // Also true for the first packet, when the filter is not configured yet.
        if (frames != filter_.framesPerPeriod()) {
            if (filter_.framesPerPeriod() != 0) {
                ++stats_.frameCountChanges;
            }
            filter_.configure(kBandwidth, frames, sampleRate_, kStartBandwidth, kStartSeconds);
            lostTrack();
        }

        if (!filter_.ready()) {
            filter_.reset(t);
            return;
        }

        filter_.update(t);
        window_.addError(filter_.error());
    }

    udp::socket& rx_;
    Ring& ring_;
    ReportQueue& reports_;
    ReceiveStats& stats_;
    const clocksync::ClockSync& clock_;
    const std::size_t bytesPerFrame_;
    const unsigned int sampleRate_;
    const Clock::time_point origin_;

    TimeFilter filter_;
    bool restartPending_ = false;
    int warmupPackets_ = 10;
    std::uint32_t session_ = 0;
    std::uint32_t expectedSeq_ = 0;
    bool stopping_ = false;
    ReceiveWindow window_;

    std::array<std::uint8_t, 8192> buf_;    // last, so the hot members share cache lines
};

struct Options {
    unsigned int outputDeviceIndex = 0;
    unsigned int senderSampleRate = kDefaultSampleRate;
    unsigned int periodSizeInFrames = kPeriodSizeInFrames;
    double outputLatencyMs = kDefaultOutputLatencyMs;
    std::string networkInterface;
    std::string multicastGroup{streaming::kDefaultAudioGroup};
    unsigned short multicastPort = streaming::kDefaultAudioPort;
};

void enumerateOutputDevices(const AudioContext& audio) {
    for (ma_uint32 deviceIndex = 0; deviceIndex < audio.playbackCount(); deviceIndex += 1) {
        const ma_device_info& info = audio.playbackInfo(deviceIndex);
        std::cout << deviceIndex << " - " << info.name
                  << (info.isDefault ? " (default)" : "") << "\n";
    }
}

// Returns an exit code if the program should end here (--help, enumeration,
// bad input), nothing if it should go on.
std::optional<int> parseOptions(int argc, char** argv, const AudioContext& audio, Options& opts) {
    CLI::App app{"Receiver"};
    argv = app.ensure_utf8(argv);

    app.add_flag("--enumOutputDevices",
        [&audio] (int64_t) {
            enumerateOutputDevices(audio);
            throw CLI::Success();
        }, "Enumerate audio output devices")
        ->trigger_on_parse();
    app.add_flag("--enumNetworkInterfaces",
        [] (int64_t) {
            net::printInterfaces(std::cout);
            throw CLI::Success();
        }, "Enumerate network interfaces")
        ->trigger_on_parse();

    app.add_option("-o,--outputDeviceIndex", opts.outputDeviceIndex, "Index of the audio output device")
        ->required();
    app.add_option("-s,--senderSampleRate", opts.senderSampleRate, "Sender sample rate")
        ->check(CLI::PositiveNumber);
    app.add_option("-f,--periodSizeInFrames", opts.periodSizeInFrames, "Period size in frames")
        ->check(CLI::PositiveNumber);
    app.add_option("--outputLatency", opts.outputLatencyMs,
                   "Milliseconds from the audio callback to the sound leaving the speaker; measure it per receiver")
        ->check(CLI::NonNegativeNumber);
    app.add_option("-n,--networkInterface", opts.networkInterface, "Network interface");
    app.add_option("-m,--multicastGroup", opts.multicastGroup, "Multicast group address")
        ->check(CLI::ValidIPV4);
    app.add_option("-p,--multicastPort", opts.multicastPort, "Multicast port");

    // What CLI11_PARSE expands to; CLI::Success from the flags above lands here too.
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }
    return std::nullopt;
}

// Returns the interface it chose, for the clock sync to use too.
std::optional<net::Interface> openSocket(udp::socket& rx, const Options& opts) {
    const auto chosen = !opts.networkInterface.empty() ? net::find(opts.networkInterface) : net::selectDefault();
    if (!chosen) {
        std::cerr << (!opts.networkInterface.empty() ? "No such interface\n" : "Ambiguous or none; name one explicitly\n");
        return std::nullopt;
    }
    std::cout << "Using network interface " << chosen->name << " " << chosen->address.to_string() << "\n";

    const udp::endpoint group(asio::ip::make_address(opts.multicastGroup), opts.multicastPort);
    net::configureReceiver(rx, group, *chosen);
    std::cout << "Receiver configured\n";
    return chosen;
}

}  // namespace

int main(int argc, char** argv) {
    AudioContext audio;
    if (!audio.init()) {
        std::cerr << "Failed to initialise the audio context\n";
        return exit_code::kAudio;
    }
    if (audio.playbackCount() == 0) {
        std::cerr << "No audio playback devices available\n";
        return exit_code::kAudio;
    }

    Options opts;
    if (const auto exitCode = parseOptions(argc, argv, audio, opts)) {
        return *exitCode;
    }

    if (opts.outputDeviceIndex >= audio.playbackCount()) {
        std::cerr << "Output device with index " << opts.outputDeviceIndex << " not available\n";
        return exit_code::kAudio;
    }

    asio::io_context io;
    udp::socket rx(io);
    const auto iface = openSocket(rx, opts);
    if (!iface) {
        return exit_code::kNetwork;
    }

    // A clock-sync slave of the sender: master time is the sender's clock.
    // Binding the socket and joining the group throw on failure.
    std::optional<clocksync::ClockSync> clock;
    try {
        clock.emplace(clocksync::Config{
            .group = udp::endpoint(asio::ip::make_address(kClockGroup), kClockPort),
            .iface = *iface,
            .nodeId = clocksync::randomNodeId(),
        }, clocksync::Role::Slave);
    } catch (const std::exception& e) {
        std::cerr << "Failed to set up the clock-sync slave: " << e.what() << "\n";
        return exit_code::kNetwork;
    }

    AudioPlayer::Config playerConfig {
        .format = ma_format_s16,
        .channels = 0,     // native
        .sampleRate = 0,   // native: the rate loop's resampler converts, not miniaudio's
        .periodSizeInFrames = opts.periodSizeInFrames,
        .periods = kNumPeriods,
    };

    Ring frameRing;
    ReportQueue reports;
    ReceiveStats stats;
    PlaybackStats playbackStats;

    const auto origin = Clock::now();

    PlaybackState state{
        .outputLatency = opts.outputLatencyMs / 1000.0,
        .ring = &frameRing,
        .reports = &reports,
        .stats = &playbackStats,
        .clock = &*clock,
        .origin = origin,
        .originSeconds = toSeconds(sinceEpoch(origin)),
    };

    AudioPlayer player;
    if (!player.open(audio, audio.playbackInfo(opts.outputDeviceIndex).id, playerConfig, onPlayback, &state)) {
        std::cerr << "Failed to open the selected output device\n";
        return exit_code::kAudio;
    }
    const unsigned int deviceRate = player.sampleRate();
    const double nominalRatio = static_cast<double>(opts.senderSampleRate) / deviceRate;
    std::cout << "playback: " << player.channels() << " ch s16 @ " << deviceRate << " Hz, "
              << "sender @ " << opts.senderSampleRate << " Hz, "
              << "output latency " << opts.outputLatencyMs << " ms\n";

    // Everything the audio callback touches has to be in place before start().
    const std::size_t bytesPerFrame = player.bytesPerFrame();
    if (!frameRing.init(bytesPerFrame)) {
        std::cerr << "Unsupported frame size: " << bytesPerFrame << " bytes\n";
        return exit_code::kAudio;
    }
    state.deviceRate = deviceRate;
    state.bytesPerFrame = bytesPerFrame;
    state.headroomWindowFrames = deviceRate;    // one second
    state.resampler.configure(player.channels(), nominalRatio);
    state.loop.configure({
        .nominalRatio = nominalRatio,
        .deviceRate = static_cast<double>(deviceRate),
        .framesPerCallback = opts.periodSizeInFrames,
        .realignFrames = kRealignMs / 1000.0 * opts.senderSampleRate,
    });

    PacketReceiver receiver(rx, frameRing, reports, stats, *clock, bytesPerFrame, opts.senderSampleRate, origin);

    // The main thread only reports and waits for signals; packets never wait for it.
    asio::io_context control;
    StatusReporter reporter(control, io, [&receiver]{ return receiver.takeSnapshot(); }, playbackStats, *clock,
                            opts.senderSampleRate, deviceRate, bytesPerFrame, origin);

    // Before playback, so the mapping can settle while the receiver is silent.
    clock->start();

    if (!player.start()) {
        std::cerr << "Failed to start the playback device\n";
        return exit_code::kAudio;
    }

    receiver.start();

    const auto packetPeriod = std::chrono::nanoseconds(
        std::chrono::seconds(opts.periodSizeInFrames)) / opts.senderSampleRate;
    std::promise<bool> realtime;
    std::future<bool> realtimeResult = realtime.get_future();
    std::thread worker([&]{
        realtime.set_value(rt::makeCurrentThreadRealtime(packetPeriod, kReceiveComputation));
        io.run();
    });
    if (!realtimeResult.get()) {
        std::cerr << "Could not give the receive thread real-time priority; it runs at normal priority\n";
    }

    reporter.start();
    runUntilShutdownSignal(control);
    reporter.stop();

    player.stop();

    // From inside the io thread, which owns the receiver.
    asio::post(io, [&receiver]{ receiver.stop(); });
    worker.join();
    clock->stop();      // after both of its readers

    printReceiveStats(stats, playbackStats);

    return 0;
}
