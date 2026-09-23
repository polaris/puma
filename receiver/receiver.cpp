
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
#include "audio_player.h"
#include "netint.h"
#include "receive_stats.h"
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
constexpr std::size_t kHeaderBytes = 16;        // seq(4) + frames(4) + timestamp(8)

constexpr std::size_t kRingFrames = 2048;
using Ring = FrameRing<kRingFrames>;
constexpr std::size_t kMaxChannels = 8;         // FrameRing takes up to 16 bytes per frame

constexpr double kDefaultBufferMarginMs = 3.0;
constexpr std::int64_t kMinTrimFrames = 48;     // hysteresis: smaller excesses are left alone
constexpr std::size_t kCrossfadeFrames = 64;
constexpr std::size_t kMaxHeadroom = kRingFrames / 2;
constexpr unsigned int kHeadroomDecaySeconds = 60;  // without underruns, before the target steps down

// CPU the receive thread needs per packet, at most; for the real-time scheduler.
constexpr auto kReceiveComputation = std::chrono::microseconds(500);

struct PlaybackState {
    unsigned int sampleRate = 0;
    std::size_t bytesPerFrame = 0;
    std::size_t minHeadroom = 0;        // floor of the target: frames left in the ring after a read, at the lowest point
    std::size_t trimWindowFrames = 0;   // how long to watch the lowest point before acting on it
    std::size_t decayFrames = 0;        // how long without underruns before the target steps down

    Ring* ring = nullptr;
    PlaybackStats* stats = nullptr;

    std::chrono::steady_clock::time_point origin;

    // Audio thread only.
    TimeFilter timeFilter{};
    bool primed = false;
    std::size_t targetHeadroom = 0;     // starts at minHeadroom, rises with underruns, decays back
    std::size_t framesSinceRaise = 0;
    std::int64_t windowMinHeadroom = std::numeric_limits<std::int64_t>::max();
    std::int64_t windowMaxHeadroom = std::numeric_limits<std::int64_t>::min();
    std::size_t windowFrames = 0;
    std::array<std::int16_t, kRingFrames * kMaxChannels> scratch{};     // for trimmed reads
};

struct PacketHeader {
    std::uint32_t seq = 0;
    std::uint32_t frames = 0;
    std::uint64_t ts = 0;

};

PacketHeader parsePacketHeader(const std::uint8_t* buf) {
    PacketHeader header;
    std::memcpy(&header.seq, buf, 4);
    std::memcpy(&header.frames, buf + 4, 4);
    std::memcpy(&header.ts, buf + 8, 8);
    return header;
}

void printReceiveStats(const ReceiveStats& stats, const PlaybackStats& playback) {
    std::cerr << "discontinuities " << stats.discontinuities
              << ", lost frames " << stats.lostFrames
              << ", conceal failures " << stats.concealFailures
              << ", resyncs " << stats.resyncs
              << ", ring drops " << stats.ringDrops
              << ", size mismatches " << stats.sizeMismatches
              << ", receive errors " << stats.receiveErrors
              << ", runt packets " << stats.runtPackets
              << ", frame count changes " << stats.frameCountChanges
              << ", underrun frames " << playback.underrunFrames.load()
              << ", trimmed frames " << playback.trimmedFrames.load()
              << "\n";

}

static void trackDeviceClock(PlaybackState& s, ma_uint32 frameCount) {
    const auto t = std::chrono::duration<double>(Clock::now() - s.origin).count();
    // Also true on the first call. miniaudio does not promise a fixed callback
    // size, and the filter assumes one, so it starts over when the size changes.
    if (frameCount != s.timeFilter.framesPerPeriod()) {
        s.timeFilter.configure(kBandwidth, frameCount, s.sampleRate);
        s.timeFilter.reset(t);
    } else {
        s.timeFilter.update(t);
        s.stats->deviceRate.store(s.timeFilter.rate(), std::memory_order_relaxed);
    }
}

// Follows the headroom over a window and acts on it at the end of each one:
//  - every read in the window came up short: the stream stopped, not jitter,
//    so prime again rather than learn from it;
//  - some reads came up short: the target was too low for this network, so
//    raise it by the worst shortfall (the underrun already added the frames);
//  - none did: after a long quiet spell, let the target step down, and trim
//    whatever sits above it.
// Returns the frames to trim now, never more than the ring holds beyond
// `frameCount`: the lowest point is no higher than the current one.
static std::size_t adaptBuffer(PlaybackState& s, std::size_t fill, std::size_t frameCount) {
    const auto headroom = static_cast<std::int64_t>(fill) - static_cast<std::int64_t>(frameCount);
    s.windowMinHeadroom = std::min(s.windowMinHeadroom, headroom);
    s.windowMaxHeadroom = std::max(s.windowMaxHeadroom, headroom);
    s.windowFrames += frameCount;
    if (s.windowFrames < s.trimWindowFrames) {
        return 0;
    }

    const std::int64_t lowest = s.windowMinHeadroom;
    const std::int64_t highest = s.windowMaxHeadroom;
    s.windowMinHeadroom = std::numeric_limits<std::int64_t>::max();
    s.windowMaxHeadroom = std::numeric_limits<std::int64_t>::min();
    s.windowFrames = 0;
    s.stats->headroom.store(lowest, std::memory_order_relaxed);

    std::size_t excess = 0;
    if (highest < 0) {
        s.primed = false;
        s.stats->headroom.store(PlaybackStats::kHeadroomUnknown, std::memory_order_relaxed);
    } else if (lowest < 0) {
        s.targetHeadroom = std::min(s.targetHeadroom + static_cast<std::size_t>(-lowest), kMaxHeadroom);
        s.framesSinceRaise = 0;
    } else {
        s.framesSinceRaise += s.trimWindowFrames;
        if (s.framesSinceRaise >= s.decayFrames && s.targetHeadroom > s.minHeadroom) {
            const auto step = static_cast<std::size_t>(kMinTrimFrames);
            s.targetHeadroom = s.targetHeadroom > s.minHeadroom + step ? s.targetHeadroom - step : s.minHeadroom;
            s.framesSinceRaise = 0;
        }
        const std::int64_t above = lowest - static_cast<std::int64_t>(s.targetHeadroom);
        if (above >= kMinTrimFrames) {
            excess = static_cast<std::size_t>(above);
        }
    }
    s.stats->targetHeadroom.store(s.targetHeadroom, std::memory_order_relaxed);
    return excess;
}

// Drops `excess` frames from the front of the ring and plays what follows,
// crossfading from the dropped frames into the kept ones so the jump does not
// click. The ring holds at least frameCount + excess frames.
static void playTrimmed(PlaybackState& s, std::int16_t* out, std::size_t frameCount, std::size_t excess) {
    const std::size_t channels = s.bytesPerFrame / sizeof(std::int16_t);
    // All of them: only this thread reads, so the fill can only have grown.
    (void)s.ring->read(s.scratch.data(), frameCount + excess);

    const std::int16_t* dropped = s.scratch.data();
    const std::int16_t* kept = s.scratch.data() + excess * channels;
    const std::size_t fade = std::min(kCrossfadeFrames, frameCount);
    for (std::size_t i = 0; i < fade; ++i) {
        const float w = static_cast<float>(i + 1) / static_cast<float>(fade + 1);
        for (std::size_t c = 0; c < channels; ++c) {
            const std::size_t k = i * channels + c;
            out[k] = static_cast<std::int16_t>(std::lrint(dropped[k] * (1.0f - w) + kept[k] * w));
        }
    }
    std::memcpy(out + fade * channels, kept + fade * channels, (frameCount - fade) * s.bytesPerFrame);

    s.stats->trimmedFrames.fetch_add(excess, std::memory_order_relaxed);
}

static void playFromRing(PlaybackState& s, std::uint8_t* out, std::size_t frameCount) {
    const std::size_t got = s.ring->read(out, frameCount);
    if (got < frameCount) {
        const std::size_t short_ = frameCount - got;
        std::memset(out + got * s.bytesPerFrame, 0, short_ * s.bytesPerFrame);
        s.stats->underrunFrames.fetch_add(short_, std::memory_order_relaxed);
    }
}

static void onPlayback(void* user, void* output, ma_uint32 frameCount) {
    auto* const s = static_cast<PlaybackState*>(user);
    trackDeviceClock(*s, frameCount);

    const std::size_t fill = s->ring->availableRead();

    // Silence until the ring holds a read plus the target; from then on,
    // underruns and trims keep it near that.
    if (!s->primed) {
        if (fill < frameCount + s->targetHeadroom) {
            std::memset(output, 0, frameCount * s->bytesPerFrame);
            return;
        }
        s->primed = true;
    }

    if (const std::size_t excess = adaptBuffer(*s, fill, frameCount); excess > 0) {
        playTrimmed(*s, static_cast<std::int16_t*>(output), frameCount, excess);
    } else {
        playFromRing(*s, static_cast<std::uint8_t*>(output), frameCount);
    }
}

class PacketReceiver {
public:
    PacketReceiver(udp::socket& rx, Ring& ring, ReceiveStats& stats, std::size_t bytesPerFrame,
                   unsigned int sampleRate, Clock::time_point origin)
    : rx_{rx}
    , ring_{ring}
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
        if (n < kHeaderBytes) {
            arm();
            ++stats_.runtPackets;
            return;
        }

        const auto arrival = Clock::now();
        const auto header = parsePacketHeader(buf_.data());

        if (discard_ > 0) {
            --discard_;
            expectedSeq_ = header.seq + 1;
            arm();
            return;
        }

        const std::uint32_t gap = header.seq - expectedSeq_;
        if (gap != 0) {
            concealGap(gap, header.frames);
        }
        expectedSeq_ = header.seq + 1;

        storePayload(header.frames, n - kHeaderBytes);

        arm();

        window_.addFill(ring_.availableRead());
        trackTiming(arrival, header.frames);
    }

    void concealGap(std::uint32_t gap, std::uint32_t frames) {
        ++stats_.discontinuities;
        const std::size_t missing = static_cast<std::size_t>(gap) * frames;

        if (missing > kRingFrames) {
            ++stats_.resyncs;
            filter_.invalidate();
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
        } else if (!ring_.write(buf_.data() + kHeaderBytes, frames)) {
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
            filter_.invalidate();
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
    ReceiveStats& stats_;
    const std::size_t bytesPerFrame_;
    const unsigned int sampleRate_;
    const Clock::time_point origin_;

    TimeFilter filter_;
    int discard_ = 10;
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
                   "Least receive buffer kept in reserve against jitter, in milliseconds; grows after underruns")
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
        .sampleRate = opts.senderSampleRate,
        .periodSizeInFrames = opts.periodSizeInFrames,
        .periods = kNumPeriods,
    };

    Ring frameRing;
    ReceiveStats stats;
    PlaybackStats playbackStats;

    const auto origin  = Clock::now();

    const auto minHeadroom = static_cast<std::size_t>(
        std::lround(opts.bufferMarginMs * opts.senderSampleRate / 1000.0));
    if (minHeadroom > kMaxHeadroom) {
        std::cerr << "Buffer margin of " << opts.bufferMarginMs << " ms does not fit the "
                  << kMaxHeadroom << " frame limit\n";
        return 2;
    }

    PlaybackState state{
        .sampleRate = opts.senderSampleRate,
        .minHeadroom = minHeadroom,
        .trimWindowFrames = opts.senderSampleRate,      // one second
        .decayFrames = kHeadroomDecaySeconds * opts.senderSampleRate,
        .stats = &playbackStats,
        .origin = origin,
        .targetHeadroom = minHeadroom,
    };

    AudioPlayer player;
    if (!player.open(audio, audio.playbackInfo(opts.outputDeviceIndex).id, playerConfig, onPlayback, &state)) {
        std::cerr << "Failed to open the selected output device\n";
        return 2;
    }
    if (player.internalSampleRate() != opts.senderSampleRate) {
        std::cerr << "Output device runs at " << player.internalSampleRate()
                  << " Hz, but the sender uses " << opts.senderSampleRate << " Hz\n";
        return 2;
    }
    std::cout << "playback: " << player.channels() << " ch s16 @ " << player.sampleRate() << " Hz, "
              << "buffer margin " << minHeadroom << " frames (" << opts.bufferMarginMs << " ms)\n";

    // Everything the audio callback touches has to be in place before start().
    const std::size_t bytesPerFrame = player.bytesPerFrame();
    if (!frameRing.init(bytesPerFrame)) {
        std::cerr << "Unsupported frame size: " << bytesPerFrame << " bytes\n";
        return 2;
    }
    state.ring = &frameRing;
    state.bytesPerFrame = bytesPerFrame;

    PacketReceiver receiver(rx, frameRing, stats, bytesPerFrame, opts.senderSampleRate, origin);

    // The main thread only reports and waits for signals; packets never wait for it.
    asio::io_context control;
    StatusReporter reporter(control, io, [&receiver]{ return receiver.takeSnapshot(); }, playbackStats,
                            opts.senderSampleRate, bytesPerFrame, origin);

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
