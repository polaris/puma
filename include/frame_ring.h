#ifndef FRAME_RING_H
#define FRAME_RING_H

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

struct Region {
    std::uint8_t *buf = nullptr;
    std::size_t len = 0;
};

class Regions {
public:
    explicit Regions(const Region region1, const Region region2)
    : region1_{region1}, region2_{region2} {
    }

    [[nodiscard]] std::size_t frames() const noexcept {
        return region1_.len + region2_.len;
    }

    [[nodiscard]] const Region& region1() const noexcept {
        return region1_;
    }

    [[nodiscard]] const Region& region2() const noexcept {
        return region2_;
    }

private:
    Region region1_, region2_;
};

template <std::size_t CapacityFrames>
class FrameRing {
    static_assert(CapacityFrames >= 2 && (CapacityFrames & (CapacityFrames - 1)) == 0,
                  "CapacityFrames must be a power of two");
    static constexpr std::size_t kMask = CapacityFrames - 1;
    static constexpr std::size_t kMinFrameBytes = 2;   // 1 ch * s16
    static constexpr std::size_t kMaxFrameBytes = 16;   // 8 ch * s16

public:
    // Call once before start(), never after.
    [[nodiscard]] bool init(std::size_t bytesPerFrame) {
        if (bytesPerFrame_ != 0 || 
            bytesPerFrame < kMinFrameBytes || 
            bytesPerFrame > kMaxFrameBytes) {
            return false;
        }
        bytesPerFrame_ = bytesPerFrame;
        return true;
    }

    [[nodiscard]] std::size_t availableRead() const noexcept {
        const std::size_t w = write_.load(std::memory_order_acquire);
        const std::size_t r = read_.load(std::memory_order_acquire);
        return w - r;
    }

    [[nodiscard]] std::size_t availableWrite() const noexcept {
        const std::size_t w = write_.load(std::memory_order_acquire);
        const std::size_t r = read_.load(std::memory_order_acquire);
        return CapacityFrames - (w - r);
    }

    [[nodiscard]] static constexpr std::size_t capacity() noexcept {
        return CapacityFrames;
    }

    [[nodiscard]] Regions acquireWrite(std::size_t requested) {
        const std::size_t w = write_.load(std::memory_order_relaxed);
        const std::size_t r = read_.load(std::memory_order_acquire);
        const std::size_t available = CapacityFrames - (w - r);
        const std::size_t offset = w & kMask;
        const std::size_t n = std::min(requested, available);
        const std::size_t len1 = std::min(n, CapacityFrames - offset);
        const std::size_t len2 = n - len1;
        return Regions{{ len1 > 0 ? frameAt(w)     : nullptr, len1 },
                       { len2 > 0 ? buffer_.data() : nullptr, len2 }};
    }

    [[nodiscard]] bool commitWrite(std::size_t written) {
        const std::size_t w = write_.load(std::memory_order_relaxed);
        const std::size_t r = read_.load(std::memory_order_acquire);
        const std::size_t available = CapacityFrames - (w - r);
        if (written > available) {
            return false;
        }
        write_.store(w + written, std::memory_order_release);
        return true;
    }

    [[nodiscard]] Regions acquireRead(std::size_t requested) {
        const std::size_t r = read_.load(std::memory_order_relaxed);
        const std::size_t w = write_.load(std::memory_order_acquire);
        const std::size_t available = w - r;
        const std::size_t offset = r & kMask;
        const std::size_t n = std::min(requested, available);
        const std::size_t len1 = std::min(n, CapacityFrames - offset);
        const std::size_t len2 = n - len1;
        return Regions{{ len1 > 0 ? frameAt(r)     : nullptr, len1 },
                       { len2 > 0 ? buffer_.data() : nullptr, len2 }};
    }

    [[nodiscard]] bool commitRead(std::size_t read) {
        const std::size_t r = read_.load(std::memory_order_relaxed);
        const std::size_t w = write_.load(std::memory_order_acquire);
        const std::size_t available = w - r;
        if (read > available) {
            return false;
        }
        read_.store(r + read, std::memory_order_release);
        return true;
    }

    // Copies `frames` frames in, or nothing at all, and returns whether it fit.
    //
    // All-or-nothing because a torn packet is worse than a dropped one: the
    // consumer would play the fragment and continue straight into the next
    // packet, with the missing frames gone and nothing recording that they
    // were. A refusal is one countable event at a packet boundary, which is
    // what the sender already does when its own ring is full. Nothing is
    // published on refusal, because acquireWrite does not move the counter.
    [[nodiscard]] bool write(const void* src, std::size_t frames) {
        const Regions regions = acquireWrite(frames);
        if (regions.frames() < frames) {
            return false;
        }

        const auto* in = static_cast<const std::uint8_t*>(src);
        const std::size_t bytes1 = regions.region1().len * bytesPerFrame_;
        const std::size_t bytes2 = regions.region2().len * bytesPerFrame_;
        if (bytes1 > 0) std::memcpy(regions.region1().buf, in, bytes1);
        if (bytes2 > 0) std::memcpy(regions.region2().buf, in + bytes1, bytes2);
        return commitWrite(frames);
    }

    // Copies up to `frames` frames out and returns how many it got.
    //
    // Clamps rather than refusing, which is the opposite of write() on purpose:
    // the audio callback has to produce a full period whatever happens, so
    // taking what is there beats taking nothing. The shortfall is the caller's
    // to pad with silence and to count. Frames beyond the returned count are
    // left untouched, not zeroed.
    [[nodiscard]] std::size_t read(void* dst, std::size_t frames) {
        const Regions regions = acquireRead(frames);

        auto* out = static_cast<std::uint8_t*>(dst);
        const std::size_t bytes1 = regions.region1().len * bytesPerFrame_;
        const std::size_t bytes2 = regions.region2().len * bytesPerFrame_;
        if (bytes1 > 0) std::memcpy(out, regions.region1().buf, bytes1);
        if (bytes2 > 0) std::memcpy(out + bytes1, regions.region2().buf, bytes2);

        const std::size_t n = regions.frames();
        return commitRead(n) ? n : 0;
    }

private:
    std::size_t bytesPerFrame_ = 0;         // runtime; frames are the unit everywhere else
    alignas(128) std::atomic<std::size_t> write_{0};
    alignas(128) std::atomic<std::size_t> read_{0};
    alignas(128) std::array<std::uint8_t, CapacityFrames * kMaxFrameBytes> buffer_{};

    [[nodiscard]] std::uint8_t* frameAt(std::size_t counter) noexcept {
        return buffer_.data() + (counter & kMask) * bytesPerFrame_;
    }
};

#endif  // FRAME_RING_H
