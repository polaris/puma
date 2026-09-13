#include <asio.hpp>

#include "netint.h"

#define MA_IMPLEMENTATION
#include <miniaudio.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <semaphore>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
using asio::ip::udp;

constexpr std::size_t kHeaderBytes = 16;    // seq(4) + frames(4) + timestamp(8)
constexpr std::size_t kMaxPayload  = 8192;  // 240 frames * 8ch * f32 + header
constexpr std::size_t kSlots       = 32;    // ~170 ms of slack at 5.4 ms periods
  
struct Slot {
    std::uint32_t length = 0;
    std::array<std::uint8_t, kMaxPayload> data{};
};
 
class PacketRing {
    static constexpr std::size_t kMask = kSlots - 1;
    static_assert((kSlots & kMask) == 0, "kSlots must be a power of two");
 
public:
    // Producer: claim a slot, fill it, then commit. No copy of the payload.
    Slot* claim() noexcept {
        const std::size_t w = write_.load(std::memory_order_relaxed);
        if (((w + 1) & kMask) == read_.load(std::memory_order_acquire)) {
            return nullptr;
        }
        return &slots_[w];
    }
    void commit() noexcept {
        const std::size_t w = write_.load(std::memory_order_relaxed);
        write_.store((w + 1) & kMask, std::memory_order_release);
    }
 
    // Consumer: peek at the front, then release it once sent.
    const Slot* front() noexcept {
        const std::size_t r = read_.load(std::memory_order_relaxed);
        if (r == write_.load(std::memory_order_acquire)) {
            return nullptr;
        }
        return &slots_[r];
    }
    void pop() noexcept {
        const std::size_t r = read_.load(std::memory_order_relaxed);
        read_.store((r + 1) & kMask, std::memory_order_release);
    }
 
    std::size_t size() const noexcept {
        const std::size_t w = write_.load(std::memory_order_acquire);
        const std::size_t r = read_.load(std::memory_order_acquire);
        return (w - r) & kMask;
    }

private:
    alignas(128) std::atomic<std::size_t> write_{0};
    alignas(128) std::atomic<std::size_t> read_{0};
    alignas(128) std::array<Slot, kSlots> slots_{};
};

struct SenderContext {
    PacketRing ring;
    std::counting_semaphore<> wake{0};
    std::atomic<bool> running{true};
    std::atomic<std::uint64_t> dropped{0};
    std::atomic<std::uint64_t> oversize{0};
    Clock::time_point origin;
    std::uint32_t sequence = 0;      // audio thread only
    std::size_t bytesPerFrame = 0;
};

void data_callback(ma_device* device, void* output, const void* input, ma_uint32 frameCount) {
    (void)output;
    const auto now = Clock::now();
    auto* ctx = static_cast<SenderContext*>(device->pUserData);
 
    const std::size_t payload = kHeaderBytes + frameCount * ctx->bytesPerFrame;
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
 
    const std::uint32_t seq = ctx->sequence++;
    const std::uint32_t n   = frameCount;
    const std::uint64_t ts  = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - ctx->origin).count());
 
    std::memcpy(slot->data.data() +  0, &seq, 4);
    std::memcpy(slot->data.data() +  4, &n,   4);
    std::memcpy(slot->data.data() +  8, &ts,  8);
    std::memcpy(slot->data.data() + kHeaderBytes, input, frameCount * ctx->bytesPerFrame);
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

    SenderContext ctx;
    ctx.origin = Clock::now();
 
    ma_device_config config    = ma_device_config_init(ma_device_type_capture);
    config.capture.format      = ma_format_s16;
    config.capture.channels    = 0;
    config.sampleRate          = 0;
    config.periodSizeInFrames  = 240;
    config.periods             = 3;
    config.performanceProfile  = ma_performance_profile_low_latency;
    config.pUserData           = &ctx;
    config.dataCallback        = data_callback;
 
    ma_device device;
    if (ma_device_init(nullptr, &config, &device) != MA_SUCCESS) {
        return 2;
    }
 
    ctx.bytesPerFrame = ma_get_bytes_per_frame(device.capture.format, device.capture.channels);
    if (kHeaderBytes + 240 * ctx.bytesPerFrame > kMaxPayload) {
        std::cerr << "payload too large for slot\n";
        ma_device_uninit(&device);
        return 3;
    }

    std::size_t highWater = 0;
 
    std::thread worker([&ctx, &tx, &highWater]() {
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
 
    if (ma_device_start(&device) != MA_SUCCESS) {
        ma_device_uninit(&device);
        return 4;
    }
    std::cin.get();
 
    ma_device_stop(&device);
    ma_device_uninit(&device);
    ctx.running.store(false);
    ctx.wake.release();
    worker.join();
    tx.close();
 
    std::cerr << "dropped " << ctx.dropped.load() << ", oversize " << ctx.oversize.load() <<  ", high water " << highWater << "\n";
    return 0;
}