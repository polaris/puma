#ifndef SPSC_RING_H
#define SPSC_RING_H

#include <atomic>
#include <array>
#include <cassert>

template <typename T, std::size_t Capacity>
class SpscRing {
    static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");
    static constexpr std::size_t kMask = Capacity - 1;
    static constexpr std::size_t kLine = std::hardware_destructive_interference_size;

public:
    bool push(const T& value) noexcept {
        const std::size_t w = write_.load(std::memory_order_relaxed);
        if (w - read_.load(std::memory_order_acquire) == Capacity) return false;
        buffer_[w & kMask] = value;
        write_.store(w + 1, std::memory_order_release);
        return true;
    }

    bool pop(T& value) noexcept {
        const std::size_t r = read_.load(std::memory_order_relaxed);
        if (r == write_.load(std::memory_order_acquire)) return false;
        value = buffer_[r & kMask];
        read_.store(r + 1, std::memory_order_release);
        return true;
    }

private:
    alignas(kLine) std::atomic<std::size_t> write_{0};
    alignas(kLine) std::atomic<std::size_t> read_{0};
    alignas(kLine) std::array<T, Capacity> buffer_{};
};

#endif  // SPSC_RING_H
