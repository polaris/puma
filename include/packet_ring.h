#ifndef PACKET_RING_H
#define PACKET_RING_H

#include <array>
#include <atomic>

constexpr std::size_t kSlots       = 32;    // ~170 ms of slack at 5.4 ms periods
constexpr std::size_t kMaxPayload  = 8192;  // 240 frames * 8ch * f32 + header

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

#endif  // PACKET_RING_H
