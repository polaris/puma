
#include <asio.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <CLI/CLI.hpp>

// time_filter.h pulls in the miniaudio declarations; the implementation block
// sits outside that header's include guard, so this has to be compiled here.
#define MA_IMPLEMENTATION
#include <miniaudio.h>

#include "audio_context.h"
#include "audio_packet.h"
#include "audio_player.h"
#include "netint.h"
#include "rate_loop.h"
#include "receive_stats.h"
#include "resampler.h"
#include "spsc_ring.h"
#include "status_reporter.h"
#include "thread_priority.h"
#include "time_filter.h"
#include "frame_ring.h"

using asio::ip::udp;
using Clock = std::chrono::steady_clock;

constexpr double kBandwidth = 0.05;
constexpr std::string_view kDefaultMulticastGroup = "239.255.0.1";
constexpr unsigned short kDefaultMulticastPort = 12345;
constexpr unsigned int kDefaultSampleRate = 48000;
constexpr unsigned int kPeriodSizeInFrames = 240;
constexpr unsigned int kNumPeriods = 3;
using streaming::kAudioPacketHeaderBytes;

// Room for the rate loop's whole target (a callback's read, a packet, the
// resampler's share and the margin) plus the jitter above it: ~170 ms.
constexpr std::size_t kRingFrames = 8192;
using Ring = FrameRing<kRingFrames>;

// Network thread -> audio thread: one (t, k) point per packet for the rate loop.
using ReportQueue = SpscRing<NetReport, 64>;

// Fixed, so the delay stays the same for the whole run: generous rather than
// tight, since underruns matter more here than latency.
constexpr double kDefaultBufferMarginMs = 20.0;
constexpr std::size_t kMaxMargin = kRingFrames / 4;   // leaves most of the ring for jitter above the target

// CPU the receive thread needs per packet, at most; for the real-time scheduler.
constexpr auto kReceiveComputation = std::chrono::microseconds(500);

struct PlaybackState {
    unsigned int deviceRate = 0;
    std::size_t bytesPerFrame = 0;
    std::size_t margin = 0;             // the rate loop's, in sender frames
    std::size_t headroomWindowFrames = 0;   // how long to watch the lowest headroom before reporting it

    Ring* ring = nullptr;
    ReportQueue* reports = nullptr;
    PlaybackStats* stats = nullptr;

    std::chrono::steady_clock::time_point origin;

    // Audio thread only.
    TimeFilter timeFilter{};
    RateLoop loop{};
    Resampler resampler{};
    std::int64_t maxMarginUsed = 0;
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
              << "\n";

}

[[nodiscard]] static double trackDeviceClock(PlaybackState& s, ma_uint32 frameCount) {
    const auto t = std::chrono::duration<double>(Clock::now() - s.origin).count();
    // Also true on the first call. miniaudio does not promise a fixed callback
    // size, and the filter assumes one, so it starts over when the size changes.
    if (frameCount != s.timeFilter.framesPerPeriod()) {
        s.timeFilter.configure(kBandwidth, frameCount, s.deviceRate);
        s.timeFilter.reset(t);
        s.loop.setFramesPerCallback(frameCount);
    } else {
        s.timeFilter.update(t);
        s.stats->deviceRate.store(s.timeFilter.rate(), std::memory_order_relaxed);
    }
    return s.timeFilter.time();
}

// The ring ran dry: start over. The loop waits until the ring holds its
// target again, which also covers a sender that stopped.
static void onUnderrun(PlaybackState& s, ma_uint32 frameCount) {
    s.loop.restart();
    s.resampler.reset();
    s.stats->underrunFrames.fetch_add(frameCount, std::memory_order_relaxed);
}

// How much of the margin the jitter used at most, while running: what the
// margin needs to be, measured rather than guessed. Beyond the margin when
// the ring ran dry.
static void trackMarginUsed(PlaybackState& s, std::int64_t headroom) {
    if (s.loop.phase() != RateLoop::Phase::Running) {
        return;     // the start is the loop's transient, not jitter
    }
    const std::int64_t used = static_cast<std::int64_t>(s.margin) - headroom;
    if (used > s.maxMarginUsed) {
        s.maxMarginUsed = used;
        s.stats->maxMarginUsed.store(used, std::memory_order_relaxed);
    }
}

// Lowest headroom (ring fill beyond what a read takes) over a window, for the status line.
static void trackHeadroom(PlaybackState& s, std::int64_t headroom, ma_uint32 frameCount) {
    s.windowMinHeadroom = std::min(s.windowMinHeadroom, headroom);
    s.windowFrames += frameCount;
    if (s.windowFrames >= s.headroomWindowFrames) {
        s.stats->headroom.store(s.windowMinHeadroom, std::memory_order_relaxed);
        s.windowMinHeadroom = std::numeric_limits<std::int64_t>::max();
        s.windowFrames = 0;
    }
}

static void publishLoop(const PlaybackState& s) {
    s.stats->loopPhase.store(static_cast<int>(s.loop.phase()), std::memory_order_relaxed);
    s.stats->correction.store(s.loop.correction(), std::memory_order_relaxed);
    s.stats->delayError.store(s.loop.error(), std::memory_order_relaxed);
}

static void playSilence(const PlaybackState& s, void* output, ma_uint32 frameCount) {
    std::memset(output, 0, frameCount * s.bytesPerFrame);
}

// The rate loop's D side (see rate_loop.h): every callback measures the delay
// error at its smoothed start time, steers the resampling ratio from it, and
// then reads exactly what the resampler needs for this callback.
static void onPlayback(void* user, void* output, ma_uint32 frameCount) {
    auto& s = *static_cast<PlaybackState*>(user);
    const double t = trackDeviceClock(s, frameCount);

    for (NetReport report; s.reports->pop(report); ) {
        s.loop.addReport(report);
    }

    const auto step = s.loop.update(t, s.ring->readPosition(), s.resampler.inputDistance(), s.ring->availableRead());
    publishLoop(s);
    if (!step.play) {
        s.resampler.reset();
        s.stats->headroom.store(PlaybackStats::kHeadroomUnknown, std::memory_order_relaxed);
        playSilence(s, output, frameCount);
        return;
    }

    if (step.trim > 0) {
        // Only this thread reads, so the fill can only have grown since.
        (void)s.ring->discard(step.trim);
        s.stats->trimmedFrames.fetch_add(step.trim, std::memory_order_relaxed);
    }

    s.resampler.setRatio(step.ratio);
    const std::size_t need = s.resampler.inputFor(frameCount);
    const std::size_t fill = s.ring->availableRead();
    const std::int64_t headroom = static_cast<std::int64_t>(fill) - static_cast<std::int64_t>(need);
    trackHeadroom(s, headroom, frameCount);
    trackMarginUsed(s, headroom);
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
                   std::size_t bytesPerFrame, unsigned int sampleRate, Clock::time_point origin)
    : rx_{rx}
    , ring_{ring}
    , reports_{reports}
    , stats_{stats}
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

        storePayload(header.frames, n - kAudioPacketHeaderBytes);

        arm();

        window_.addFill(ring_.availableRead());
        trackTiming(arrival, header.frames);
        report();
    }

    // The timing starts over, and with it the rate loop, which is fed from it.
    void lostTrack() {
        filter_.invalidate();
        restartPending_ = true;
    }

    // The rate loop's N side (see rate_loop.h): when the next packet is due,
    // and where the ring's write position will be once it is in.
    void report() {
        if (reports_.push({filter_.nextTime(), ring_.writePosition() + filter_.framesPerPeriod(), restartPending_})) {
            restartPending_ = false;
        }
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

    void storePayload(std::uint32_t frames, std::size_t payloadBytes) {
        if (payloadBytes != frames * bytesPerFrame_) {
            ++stats_.sizeMismatches;
            stats_.lastMismatchBytes = payloadBytes;
            stats_.lastMismatchFrames = frames;
        } else if (!ring_.write(buf_.data() + kAudioPacketHeaderBytes, frames)) {
            ++stats_.ringDrops;                // whole packet or nothing
        }
    }

    void trackTiming(Clock::time_point arrival, std::uint32_t frames) {
        const double t = std::chrono::duration<double>(arrival - origin_).count();

        // Also true for the first packet, when the filter is not configured yet.
        if (frames != filter_.framesPerPeriod()) {
            if (filter_.framesPerPeriod() != 0) {
                ++stats_.frameCountChanges;
            }
            filter_.configure(kBandwidth, frames, sampleRate_);
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
    double bufferMarginMs = kDefaultBufferMarginMs;
    std::string networkInterface;
    std::string multicastGroup{kDefaultMulticastGroup};
    unsigned short multicastPort = kDefaultMulticastPort;
};

void enumerateOutputDevices(const AudioContext& audio) {
    for (ma_uint32 deviceIndex = 0; deviceIndex < audio.playbackCount(); deviceIndex += 1) {
        const ma_device_info& info = audio.playbackInfo(deviceIndex);
        std::cout << deviceIndex << " - " << info.name
                  << (info.isDefault ? " (default)" : "") << "\n";
    }
}

void enumerateNetworkInterfaces() {
    const auto all = net::enumerate();
    std::cout << "interfaces:\n";
    for (const auto &i : all) {
        std::cout << "  " << i.name << "  " << i.address.to_string() << "  idx=" << i.index;
        if (!i.description.empty()) {
            std::cout << "  (" << i.description << ")";
        }
        std::cout << "\n";
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
            enumerateNetworkInterfaces();
            throw CLI::Success();
        }, "Enumerate network interfaces")
        ->trigger_on_parse();

    app.add_option("-o,--outputDeviceIndex", opts.outputDeviceIndex, "Index of the audio output device")
        ->required();
    app.add_option("-s,--senderSampleRate", opts.senderSampleRate, "Sender sample rate")
        ->check(CLI::PositiveNumber);
    app.add_option("-f,--periodSizeInFrames", opts.periodSizeInFrames, "Period size in frames")
        ->check(CLI::PositiveNumber);
    app.add_option("-b,--bufferMargin", opts.bufferMarginMs,
                   "Receive buffer kept in reserve against network jitter, in milliseconds; fixed for the run")
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

bool openSocket(udp::socket& rx, const Options& opts) {
    const auto chosen = !opts.networkInterface.empty() ? net::find(opts.networkInterface) : net::selectDefault();
    if (!chosen) {
        std::cerr << (!opts.networkInterface.empty() ? "No such interface\n" : "Ambiguous or none; name one explicitly\n");
        return false;
    }
    std::cout << "Using network interface " << chosen->name << " " << chosen->address.to_string() << "\n";

    const udp::endpoint group(asio::ip::make_address(opts.multicastGroup), opts.multicastPort);
    net::configureReceiver(rx, group, *chosen);
    std::cout << "Receiver configured\n";
    return true;
}

// Runs `control` on the calling thread until SIGINT or SIGTERM.
void runUntilShutdownSignal(asio::io_context& control) {
    asio::signal_set signals{control, SIGINT, SIGTERM};
    signals.async_wait([&](auto, int) {
        control.stop();
    });
    control.run();
}

int main(int argc, char** argv) {
    AudioContext audio;
    if (!audio.init()) {
        std::cerr << "Failed to initialise the audio context\n";
        return 2;
    }
    if (audio.playbackCount() == 0) {
        std::cerr << "No audio playback devices available\n";
        return 2;
    }

    Options opts;
    if (const auto exitCode = parseOptions(argc, argv, audio, opts)) {
        return *exitCode;
    }

    if (opts.outputDeviceIndex >= audio.playbackCount()) {
        std::cerr << "Output device with index " << opts.outputDeviceIndex << " not available\n";
        return 2;
    }

    asio::io_context io;
    udp::socket rx(io);
    if (!openSocket(rx, opts)) {
        return 1;
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

    const auto origin  = Clock::now();

    // In sender frames: it is kept in the ring, ahead of the resampler.
    const auto margin = static_cast<std::size_t>(
        std::lround(opts.bufferMarginMs * opts.senderSampleRate / 1000.0));
    if (margin > kMaxMargin) {
        std::cerr << "Buffer margin of " << opts.bufferMarginMs << " ms does not fit the "
                  << kMaxMargin << " frame limit\n";
        return 2;
    }

    PlaybackState state{
        .margin = margin,
        .ring = &frameRing,
        .reports = &reports,
        .stats = &playbackStats,
        .origin = origin,
    };

    AudioPlayer player;
    if (!player.open(audio, audio.playbackInfo(opts.outputDeviceIndex).id, playerConfig, onPlayback, &state)) {
        std::cerr << "Failed to open the selected output device\n";
        return 2;
    }
    const unsigned int deviceRate = player.sampleRate();
    const double nominalRatio = static_cast<double>(opts.senderSampleRate) / deviceRate;
    std::cout << "playback: " << player.channels() << " ch s16 @ " << deviceRate << " Hz, "
              << "sender @ " << opts.senderSampleRate << " Hz, "
              << "buffer margin " << margin << " frames (" << opts.bufferMarginMs << " ms)\n";

    // Everything the audio callback touches has to be in place before start().
    const std::size_t bytesPerFrame = player.bytesPerFrame();
    if (!frameRing.init(bytesPerFrame)) {
        std::cerr << "Unsupported frame size: " << bytesPerFrame << " bytes\n";
        return 2;
    }
    state.deviceRate = deviceRate;
    state.bytesPerFrame = bytesPerFrame;
    state.headroomWindowFrames = deviceRate;    // one second
    state.resampler.configure(player.channels(), nominalRatio);
    state.loop.configure({
        .nominalRatio = nominalRatio,
        .deviceRate = static_cast<double>(deviceRate),
        .framesPerCallback = opts.periodSizeInFrames,
        .resamplerDelay = static_cast<double>(state.resampler.halfLength()),
        .margin = static_cast<double>(margin),
    });
    playbackStats.margin.store(margin, std::memory_order_relaxed);

    PacketReceiver receiver(rx, frameRing, reports, stats, bytesPerFrame, opts.senderSampleRate, origin);

    // The main thread only reports and waits for signals; packets never wait for it.
    asio::io_context control;
    StatusReporter reporter(control, io, [&receiver]{ return receiver.takeSnapshot(); }, playbackStats,
                            opts.senderSampleRate, deviceRate, bytesPerFrame, origin);

    if (!player.start()) {
        std::cerr << "Failed to start the playback device\n";
        return 2;
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

    printReceiveStats(stats, playbackStats);

    return 0;
}
