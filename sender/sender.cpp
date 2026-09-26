#include <asio.hpp>

// audio_recorder.h pulls in the miniaudio declarations; the implementation
// block sits outside that header's include guard, so this has to be compiled
// here.
#define MA_IMPLEMENTATION
#include <miniaudio.h>

#include "audio_context.h"
#include "audio_packet.h"
#include "audio_recorder.h"
#include "netint.h"
#include "packet_ring.h"
#include "thread_priority.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <iostream>
#include <optional>
#include <random>
#include <semaphore>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
using asio::ip::udp;

using streaming::kAudioPacketHeaderBytes;

constexpr ma_uint32   kPeriodSizeInFrames = 240;
constexpr ma_uint32   kNumPeriods = 3;
constexpr ma_uint32   kCaptureDeviceIndex = 0;      // no option for this yet

// CPU the send thread needs per packet, at most; for the real-time scheduler.
constexpr auto kSendComputation = std::chrono::microseconds(500);

struct SenderContext {
    PacketRing ring;
    std::counting_semaphore<> wake{0};
    std::atomic<bool> running{true};
    std::atomic<std::uint64_t> dropped{0};
    std::atomic<std::uint64_t> oversize{0};
    Clock::time_point origin;
    std::uint32_t session = 0;       // new for every run, so receivers can tell a restart from loss
    std::uint32_t sequence = 0;      // audio thread only
    std::size_t bytesPerFrame = 0;
};

void data_callback(void* user, const void* input, ma_uint32 frameCount) {
    auto* ctx = static_cast<SenderContext*>(user);
 
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
        .t       = static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count()),
    };
    std::uint8_t encoded[kAudioPacketHeaderBytes];
    streaming::encodeAudioPacketHeader(header, encoded);

    std::memcpy(slot->data.data(), encoded, kAudioPacketHeaderBytes);
    std::memcpy(slot->data.data() + kAudioPacketHeaderBytes, input, frameCount * ctx->bytesPerFrame);
    slot->length = static_cast<std::uint32_t>(payload);
 
    ctx->ring.commit();
    ctx->wake.release();
}

int main(int argc, char** argv) {
    const auto all = net::enumerate();
    std::cout << "interfaces:\n";
    for (const auto& i : all) {
        std::cout << "  " << i.name << "  " << i.address.to_string()
                  << "  idx=" << i.index;
        if (!i.description.empty()) std::cout << "  (" << i.description << ")";
        std::cout << "\n";
    }
    const auto chosen = argc > 1 ? net::find(argv[1]) : net::selectDefault();
    if (!chosen) {
        std::cerr << (argc > 1 ? "no such interface\n"
                               : "ambiguous or none; name one explicitly\n");
        return 1;
    }
    std::cout << "using " << chosen->name << " " << chosen->address.to_string() << "\n";

    const asio::ip::udp::endpoint group(asio::ip::make_address("239.255.0.1"), 12345);
    asio::io_context io;
    asio::ip::udp::socket tx(io);
    net::configureSender(tx, group, *chosen, {.hops = 1, .loopback = true});
    std::cout << "Sender configured\n";

    AudioContext audio;
    if (!audio.init()) {
        std::cerr << "Failed to initialise the audio context\n";
        return 2;
    }
    if (audio.captureCount() == 0) {
        std::cerr << "No audio capture devices available\n";
        return 2;
    }
    for (ma_uint32 deviceIndex = 0; deviceIndex < audio.captureCount(); deviceIndex += 1) {
        const ma_device_info& info = audio.captureInfo(deviceIndex);
        std::cout << deviceIndex << " - " << info.name
                  << (info.isDefault ? " (default)" : "") << "\n";
    }

    SenderContext ctx;
    ctx.origin = Clock::now();
    ctx.session = std::random_device{}();

    AudioRecorder::Config recorderConfig;
    recorderConfig.format             = ma_format_s16;
    recorderConfig.channels           = 0;      // native
    recorderConfig.sampleRate         = 0;      // native: the sender defines the rate
    recorderConfig.periodSizeInFrames = kPeriodSizeInFrames;
    recorderConfig.periods            = kNumPeriods;

    AudioRecorder recorder;
    if (!recorder.open(audio, audio.captureInfo(kCaptureDeviceIndex).id, recorderConfig,
                       data_callback, &ctx)) {
        std::cerr << "Failed to open the capture device\n";
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

    // The callback reads bytesPerFrame, so it has to be set before start().
    ctx.bytesPerFrame = recorder.bytesPerFrame();
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
 
    std::cerr << "dropped " << ctx.dropped.load() << ", oversize " << ctx.oversize.load() <<  ", high water " << highWater << "\n";
    return 0;
}