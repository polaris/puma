#include <asio.hpp>
#include <CLI/CLI.hpp>

// audio_recorder.h pulls in the miniaudio declarations; the implementation
// block sits outside that header's include guard, so this has to be compiled
// here.
#define MA_IMPLEMENTATION
#include <miniaudio.h>

#include "audio_context.h"
#include "audio_packet.h"
#include "audio_recorder.h"
#include "clock_sync.h"
#include "netint.h"
#include "packet_ring.h"
#include "thread_priority.h"
#include "time_filter.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <future>
#include <iostream>
#include <optional>
#include <random>
#include <semaphore>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
using asio::ip::udp;

using streaming::kAudioPacketHeaderBytes;

constexpr ma_uint32   kPeriodSizeInFrames = 240;
constexpr ma_uint32   kNumPeriods = 3;

// CPU the send thread needs per packet, at most; for the real-time scheduler.
constexpr auto kSendComputation = std::chrono::microseconds(500);

// From capture to the receivers' speakers. It has to cover the worst network
// delay plus each receiver's output latency; what is left, receivers spend
// waiting in their ring.
constexpr double kDefaultLatencyMs = 100.0;

// The capture clock's smoothing: fast at first, so the timeline settles
// quickly, then slow, so the stamps carry as little callback jitter as possible.
constexpr double kStartBandwidth = 1.0;    // Hz
constexpr double kBandwidth = 0.05;        // Hz
constexpr double kStartSeconds = 4.0;

constexpr std::string_view kAudioGroup = "239.255.0.1";
constexpr unsigned short kAudioPort = 12345;
constexpr std::string_view kClockGroup = "239.255.0.2";    // not the audio group
constexpr unsigned short kClockPort = 12346;

struct SenderContext {
    PacketRing ring;
    std::counting_semaphore<> wake{0};
    std::atomic<bool> running{true};
    std::atomic<std::uint64_t> dropped{0};
    std::atomic<std::uint64_t> oversize{0};
    Clock::time_point origin;
    std::uint64_t originNs = 0;      // origin in master time: ns since the steady clock's epoch
    double latency = 0.0;            // L, seconds from capture to the receivers' speakers
    unsigned int sampleRate = 0;
    std::uint32_t session = 0;       // new for every run, so receivers can tell a restart from loss
    std::size_t bytesPerFrame = 0;

    // Audio thread only.
    std::uint32_t sequence = 0;
    TimeFilter filter{};
};

// Smoothed start time of this callback, in seconds since origin. The raw
// callback times are bursty; every receiver steers toward the stamps, so they
// have to lie on a smooth line.
[[nodiscard]] static double trackCaptureClock(SenderContext& ctx, ma_uint32 frameCount) {
    const double t = std::chrono::duration<double>(Clock::now() - ctx.origin).count();

    // Also true on the first call. The filter assumes a fixed callback size,
    // so a change starts it over, and a new session with it: receivers then
    // re-align to the new timeline instead of following a jump in it.
    if (frameCount != ctx.filter.framesPerPeriod()) {
        if (ctx.filter.framesPerPeriod() != 0) {
            ++ctx.session;
        }
        ctx.filter.configure(kBandwidth, frameCount, ctx.sampleRate, kStartBandwidth, kStartSeconds);
        ctx.filter.reset(t);
        return ctx.filter.time();
    }

    ctx.filter.update(t);
    return ctx.filter.time();
}

void data_callback(void* user, const void* input, ma_uint32 frameCount) {
    auto* ctx = static_cast<SenderContext*>(user);

    // First, so the timeline runs on through dropped packets.
    const double captured = trackCaptureClock(*ctx, frameCount);

    const std::size_t payload = kAudioPacketHeaderBytes + frameCount * ctx->bytesPerFrame;
    if (payload > kMaxPayload) {
        ctx->oversize.fetch_add(1, std::memory_order_relaxed);
        return;
    }
 
    Slot* slot = ctx->ring.claim();
    if (!slot) {
        ctx->dropped.fetch_add(1, std::memory_order_relaxed);
        ++ctx->sequence;
        return;
    }
 
    const streaming::AudioPacketHeader header{
        .session = ctx->session,
        .seq     = ctx->sequence++,
        .frames  = frameCount,
        .t       = ctx->originNs + static_cast<std::uint64_t>(std::llround((captured + ctx->latency) * 1e9)),
    };
    std::uint8_t encoded[kAudioPacketHeaderBytes];
    streaming::encodeAudioPacketHeader(header, encoded);

    std::memcpy(slot->data.data(), encoded, kAudioPacketHeaderBytes);
    std::memcpy(slot->data.data() + kAudioPacketHeaderBytes, input, frameCount * ctx->bytesPerFrame);
    slot->length = static_cast<std::uint32_t>(payload);
 
    ctx->ring.commit();
    ctx->wake.release();
}

struct Options {
    unsigned int inputDeviceIndex = 0;
    std::string networkInterface;
    double latencyMs = kDefaultLatencyMs;
};

void enumerateInputDevices(const AudioContext& audio) {
    for (ma_uint32 deviceIndex = 0; deviceIndex < audio.captureCount(); deviceIndex += 1) {
        const ma_device_info& info = audio.captureInfo(deviceIndex);
        std::cout << deviceIndex << " - " << info.name
                  << (info.isDefault ? " (default)" : "") << "\n";
    }
}

void enumerateNetworkInterfaces() {
    const auto all = net::enumerate();
    std::cout << "interfaces:\n";
    for (const auto& i : all) {
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
    CLI::App app{"Sender"};
    argv = app.ensure_utf8(argv);

    app.add_flag("--enumInputDevices",
        [&audio] (int64_t) {
            enumerateInputDevices(audio);
            throw CLI::Success();
        }, "Enumerate audio input devices")
        ->trigger_on_parse();
    app.add_flag("--enumNetworkInterfaces",
        [] (int64_t) {
            enumerateNetworkInterfaces();
            throw CLI::Success();
        }, "Enumerate network interfaces")
        ->trigger_on_parse();

    app.add_option("-i,--inputDeviceIndex", opts.inputDeviceIndex, "Index of the audio input device")
        ->required();
    app.add_option("-n,--networkInterface", opts.networkInterface, "Network interface");
    app.add_option("-L,--latency", opts.latencyMs,
                   "Milliseconds from capture to the receivers' speakers, the same for every receiver")
        ->check(CLI::PositiveNumber);

    // What CLI11_PARSE expands to; CLI::Success from the flags above lands here too.
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }
    return std::nullopt;
}

// Returns the interface it chose, for the clock sync to use too.
std::optional<net::Interface> openSocket(udp::socket& tx, const Options& opts) {
    const auto chosen = !opts.networkInterface.empty() ? net::find(opts.networkInterface) : net::selectDefault();
    if (!chosen) {
        std::cerr << (!opts.networkInterface.empty() ? "No such interface\n" : "Ambiguous or none; name one explicitly\n");
        return std::nullopt;
    }
    std::cout << "Using network interface " << chosen->name << " " << chosen->address.to_string() << "\n";

    const udp::endpoint group(asio::ip::make_address(kAudioGroup), kAudioPort);
    net::configureSender(tx, group, *chosen, {.hops = 1, .loopback = true});
    std::cout << "Sender configured\n";
    return chosen;
}

int main(int argc, char** argv) {
    AudioContext audio;
    if (!audio.init()) {
        std::cerr << "Failed to initialise the audio context\n";
        return 2;
    }
    if (audio.captureCount() == 0) {
        std::cerr << "No audio capture devices available\n";
        return 2;
    }

    Options opts;
    if (const auto exitCode = parseOptions(argc, argv, audio, opts)) {
        return *exitCode;
    }

    if (opts.inputDeviceIndex >= audio.captureCount()) {
        std::cerr << "Input device with index " << opts.inputDeviceIndex << " not available\n";
        return 2;
    }

    asio::io_context io;
    udp::socket tx(io);
    const auto iface = openSocket(tx, opts);
    if (!iface) {
        return 1;
    }

    // The sender is the clock-sync master: its steady clock is master time, so
    // the stamps need no conversion. Binding the socket and joining the group
    // throw on failure.
    std::optional<clocksync::ClockSync> clockMaster;
    try {
        std::random_device rd;
        clockMaster.emplace(clocksync::Config{
            .group = asio::ip::udp::endpoint(asio::ip::make_address(kClockGroup), kClockPort),
            .iface = *iface,
            .nodeId = (static_cast<std::uint64_t>(rd()) << 32) | rd(),
        }, clocksync::Role::Master);
    } catch (const std::exception& e) {
        std::cerr << "Failed to set up the clock-sync master: " << e.what() << "\n";
        return 1;
    }
    std::cout << "clock-sync master on " << kClockGroup << ":" << kClockPort << "\n";

    SenderContext ctx;
    ctx.origin = Clock::now();
    ctx.originNs = toNanos(ctx.origin);
    ctx.latency = opts.latencyMs / 1000.0;
    ctx.session = std::random_device{}();

    AudioRecorder::Config recorderConfig;
    recorderConfig.format             = ma_format_s16;
    recorderConfig.channels           = 0;      // native
    recorderConfig.sampleRate         = 0;      // native: the sender defines the rate
    recorderConfig.periodSizeInFrames = kPeriodSizeInFrames;
    recorderConfig.periods            = kNumPeriods;

    AudioRecorder recorder;
    if (!recorder.open(audio, audio.captureInfo(opts.inputDeviceIndex).id, recorderConfig,
                       data_callback, &ctx)) {
        std::cerr << "Failed to open the selected input device\n";
        return 2;
    }

    // Asking for the native rate should get it untouched. If it did not, a
    // resampler sits between the ADC and the callback, and the timestamps the
    // receiver recovers its clock from would describe the resampler, not the
    // capture hardware.
    if (recorder.sampleRate() != recorder.internalSampleRate()) {
        std::cerr << "Capture device runs at " << recorder.internalSampleRate()
                  << " Hz but delivers " << recorder.sampleRate() << " Hz\n";
        return 2;
    }
    std::cout << "capture: " << recorder.channels() << " ch s16 @ " << recorder.sampleRate()
              << " Hz (run the receiver with -s " << recorder.sampleRate() << ")\n";

    std::cout << "latency " << opts.latencyMs << " ms\n";

    // The callback reads these, so they have to be set before start().
    ctx.bytesPerFrame = recorder.bytesPerFrame();
    ctx.sampleRate = recorder.sampleRate();
    if (kAudioPacketHeaderBytes + kPeriodSizeInFrames * ctx.bytesPerFrame > kMaxPayload) {
        std::cerr << "payload too large for slot\n";
        return 3;
    }

    std::size_t highWater = 0;
 
    const auto packetPeriod = std::chrono::nanoseconds(
        std::chrono::seconds(kPeriodSizeInFrames)) / recorder.sampleRate();
    std::promise<bool> realtime;
    std::future<bool> realtimeResult = realtime.get_future();

    std::thread worker([&ctx, &tx, &highWater, &realtime, packetPeriod]() {
        realtime.set_value(rt::makeCurrentThreadRealtime(packetPeriod, kSendComputation));
        while (ctx.running.load(std::memory_order_relaxed)) {
            ctx.wake.acquire();
            const auto d = ctx.ring.size();
            if (d > highWater) {
                highWater = d;
            }
            while (const Slot* slot = ctx.ring.front()) {
                asio::error_code ec;
                tx.send(asio::buffer(slot->data.data(), slot->length), 0, ec);
                if (ec) {
                    std::cerr << "send: " << ec.message() << "\n";
                }
                ctx.ring.pop();
            }
        }
    });
 
    if (!realtimeResult.get()) {
        std::cerr << "Could not give the send thread real-time priority; it runs at normal priority\n";
    }

    // Before the first stamp, so receivers can lock to master time while the
    // capture device starts.
    clockMaster->start();

    // Unlike the receiver, the worker has to exist before the device starts:
    // it is the consumer the callback hands slots to, so starting first would
    // drop the opening packets. That is what makes this failure path join.
    if (!recorder.start()) {
        std::cerr << "Failed to start the capture device\n";
        ctx.running.store(false);
        ctx.wake.release();
        worker.join();
        return 4;
    }
    std::cin.get();

    // Producer first, then the consumer it feeds.
    recorder.stop();
    ctx.running.store(false);
    ctx.wake.release();
    worker.join();
    tx.close();
    clockMaster->stop();
 
    std::cerr << "dropped " << ctx.dropped.load() << ", oversize " << ctx.oversize.load() <<  ", high water " << highWater << "\n";
    return 0;
}