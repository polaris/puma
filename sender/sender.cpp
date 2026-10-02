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
#include "send_stats.h"
#include "shutdown_signal.h"
#include "stream_defaults.h"
#include "thread_priority.h"
#include "time_filter.h"

#include <algorithm>
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
using streaming::kBandwidth;
using streaming::kClockGroup;
using streaming::kClockPort;
using streaming::kNumPeriods;
using streaming::kPeriodSizeInFrames;
using streaming::kStartBandwidth;
using streaming::kStartSeconds;

// CPU the send thread needs per packet, at most; for the real-time scheduler.
constexpr auto kSendComputation = std::chrono::microseconds(500);

// From capture to the receivers' speakers. It has to cover the worst network
// delay plus each receiver's output latency; what is left, receivers spend
// waiting in their ring.
constexpr double kDefaultLatencyMs = 100.0;

// Audio thread -> send thread: one encoded packet per capture callback.
using PacketsReady = std::counting_semaphore<>;

struct CaptureState {
    unsigned int sampleRate = 0;
    std::size_t bytesPerFrame = 0;
    double latency = 0.0;               // L, seconds from capture to the receivers' speakers

    PacketRing* ring = nullptr;
    PacketsReady* ready = nullptr;
    CaptureStats* stats = nullptr;

    Clock::time_point origin;
    std::uint64_t originNs = 0;         // origin in master time: ns since the steady clock's epoch

    // Audio thread only.
    std::uint32_t session = 0;          // new for every run, so receivers can tell a restart from loss
    std::uint32_t sequence = 0;
    TimeFilter timeFilter{};
};

void printSendStats(const SendStats& stats, const CaptureStats& capture) {
    std::cerr << "dropped packets " << capture.droppedPackets.load()
              << ", oversize packets " << capture.oversizePackets.load()
              << ", send errors " << stats.sendErrors
              << ", ring high water " << stats.ringHighWater
              << "\n";
    if (stats.lastSendError) {
        std::cerr << "last send error: " << stats.lastSendError.message() << "\n";
    }
}

// Smoothed start time of this callback, in seconds since origin. The raw
// callback times are bursty; every receiver steers toward the stamps, so they
// have to lie on a smooth line, with as little callback jitter as possible.
[[nodiscard]] static double trackCaptureClock(CaptureState& s, ma_uint32 frameCount) {
    const double t = std::chrono::duration<double>(Clock::now() - s.origin).count();

    // Also true on the first call. The filter assumes a fixed callback size,
    // so a change starts it over, and a new session with it: receivers then
    // re-align to the new timeline instead of following a jump in it.
    if (frameCount != s.timeFilter.framesPerPeriod()) {
        if (s.timeFilter.framesPerPeriod() != 0) {
            ++s.session;
        }
        s.timeFilter.configure(kBandwidth, frameCount, s.sampleRate, kStartBandwidth, kStartSeconds);
        s.timeFilter.reset(t);
        return s.timeFilter.time();
    }

    s.timeFilter.update(t);
    return s.timeFilter.time();
}

// Every callback stamps its frames with the time they are to be heard and
// hands them to the send thread as one packet.
static void onCapture(void* user, const void* input, ma_uint32 frameCount) {
    auto& s = *static_cast<CaptureState*>(user);

    // First, so the timeline runs on through dropped packets.
    const double captured = trackCaptureClock(s, frameCount);

    const std::size_t payload = kAudioPacketHeaderBytes + frameCount * s.bytesPerFrame;
    if (payload > kMaxPayload) {
        s.stats->oversizePackets.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    Slot* slot = s.ring->claim();
    if (!slot) {
        s.stats->droppedPackets.fetch_add(1, std::memory_order_relaxed);
        ++s.sequence;
        return;
    }

    const streaming::AudioPacketHeader header{
        .session = s.session,
        .seq     = s.sequence++,
        .frames  = frameCount,
        .t       = s.originNs + static_cast<std::uint64_t>(std::llround((captured + s.latency) * 1e9)),
    };
    std::uint8_t encoded[kAudioPacketHeaderBytes];
    streaming::encodeAudioPacketHeader(header, encoded);

    std::memcpy(slot->data.data(), encoded, kAudioPacketHeaderBytes);
    std::memcpy(slot->data.data() + kAudioPacketHeaderBytes, input, frameCount * s.bytesPerFrame);
    slot->length = static_cast<std::uint32_t>(payload);

    s.ring->commit();
    s.ready->release();
}

// Sends what the capture callback queues, on a thread of its own: a blocking
// send per packet, woken once per packet.
class PacketSender {
public:
    PacketSender(udp::socket& tx, PacketRing& ring, PacketsReady& ready, SendStats& stats)
    : tx_{tx}
    , ring_{ring}
    , ready_{ready}
    , stats_{stats} {
    }

    PacketSender(const PacketSender&) = delete;
    PacketSender& operator=(const PacketSender&) = delete;

    // The send thread's body; returns once stop() is called.
    void run() {
        while (running_.load(std::memory_order_relaxed)) {
            ready_.acquire();
            stats_.ringHighWater = std::max(stats_.ringHighWater, ring_.size());
            while (const Slot* slot = ring_.front()) {
                send(*slot);
                ring_.pop();
            }
        }
    }

    // From any thread. Packets queued after the last wake-up are not sent.
    void stop() {
        running_.store(false, std::memory_order_relaxed);
        ready_.release();
    }

private:
    void send(const Slot& slot) {
        asio::error_code ec;
        tx_.send(asio::buffer(slot.data.data(), slot.length), 0, ec);
        if (ec) {
            ++stats_.sendErrors;
            stats_.lastSendError = ec;
        }
    }

    udp::socket& tx_;
    PacketRing& ring_;
    PacketsReady& ready_;
    SendStats& stats_;

    std::atomic<bool> running_{true};
};

struct Options {
    unsigned int inputDeviceIndex = 0;
    double latencyMs = kDefaultLatencyMs;
    std::string networkInterface;
    std::string multicastGroup{streaming::kDefaultAudioGroup};
    unsigned short multicastPort = streaming::kDefaultAudioPort;
};

void enumerateInputDevices(const AudioContext& audio) {
    for (ma_uint32 deviceIndex = 0; deviceIndex < audio.captureCount(); deviceIndex += 1) {
        const ma_device_info& info = audio.captureInfo(deviceIndex);
        std::cout << deviceIndex << " - " << info.name
                  << (info.isDefault ? " (default)" : "") << "\n";
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
            net::printInterfaces(std::cout);
            throw CLI::Success();
        }, "Enumerate network interfaces")
        ->trigger_on_parse();

    app.add_option("-i,--inputDeviceIndex", opts.inputDeviceIndex, "Index of the audio input device")
        ->required();
    app.add_option("-L,--latency", opts.latencyMs,
                   "Milliseconds from capture to the receivers' speakers, the same for every receiver")
        ->check(CLI::PositiveNumber);
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
std::optional<net::Interface> openSocket(udp::socket& tx, const Options& opts) {
    const auto chosen = !opts.networkInterface.empty() ? net::find(opts.networkInterface) : net::selectDefault();
    if (!chosen) {
        std::cerr << (!opts.networkInterface.empty() ? "No such interface\n" : "Ambiguous or none; name one explicitly\n");
        return std::nullopt;
    }
    std::cout << "Using network interface " << chosen->name << " " << chosen->address.to_string() << "\n";

    const udp::endpoint group(asio::ip::make_address(opts.multicastGroup), opts.multicastPort);
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
        clockMaster.emplace(clocksync::Config{
            .group = asio::ip::udp::endpoint(asio::ip::make_address(kClockGroup), kClockPort),
            .iface = *iface,
            .nodeId = clocksync::randomNodeId(),
        }, clocksync::Role::Master);
    } catch (const std::exception& e) {
        std::cerr << "Failed to set up the clock-sync master: " << e.what() << "\n";
        return 1;
    }
    std::cout << "clock-sync master on " << kClockGroup << ":" << kClockPort << "\n";

    AudioRecorder::Config recorderConfig {
        .format = ma_format_s16,
        .channels = 0,      // native
        .sampleRate = 0,    // native: the sender defines the rate
        .periodSizeInFrames = kPeriodSizeInFrames,
        .periods = kNumPeriods,
    };

    PacketRing packetRing;
    PacketsReady ready{0};
    SendStats stats;
    CaptureStats captureStats;

    const auto origin = Clock::now();

    CaptureState state{
        .latency = opts.latencyMs / 1000.0,
        .ring = &packetRing,
        .ready = &ready,
        .stats = &captureStats,
        .origin = origin,
        .originNs = toNanos(origin),
        .session = std::random_device{}(),
    };

    AudioRecorder recorder;
    if (!recorder.open(audio, audio.captureInfo(opts.inputDeviceIndex).id, recorderConfig, onCapture, &state)) {
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

    // Everything the audio callback touches has to be in place before start().
    const std::size_t bytesPerFrame = recorder.bytesPerFrame();
    if (kAudioPacketHeaderBytes + kPeriodSizeInFrames * bytesPerFrame > kMaxPayload) {
        std::cerr << "payload too large for slot\n";
        return 3;
    }
    state.sampleRate = recorder.sampleRate();
    state.bytesPerFrame = bytesPerFrame;

    PacketSender sender(tx, packetRing, ready, stats);

    const auto packetPeriod = std::chrono::nanoseconds(
        std::chrono::seconds(kPeriodSizeInFrames)) / recorder.sampleRate();
    std::promise<bool> realtime;
    std::future<bool> realtimeResult = realtime.get_future();
    std::thread worker([&]{
        realtime.set_value(rt::makeCurrentThreadRealtime(packetPeriod, kSendComputation));
        sender.run();
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
        sender.stop();
        worker.join();
        return 4;
    }

    // The main thread only waits for signals; packets never wait for it.
    asio::io_context control;
    runUntilShutdownSignal(control);

    // Producer first, then the consumer it feeds.
    recorder.stop();
    sender.stop();
    worker.join();
    tx.close();
    clockMaster->stop();

    printSendStats(stats, captureStats);

    return 0;
}
