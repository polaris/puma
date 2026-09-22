#include "frame_ring.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t kCapacity = 16;
std::unique_ptr<FrameRing<kCapacity>> makeRing() { return std::make_unique<FrameRing<kCapacity>>(); }

std::uint8_t patternByte(std::size_t streamBytePos) {
    return static_cast<std::uint8_t>(streamBytePos % 251);
}

class PatternDriver {
public:
    PatternDriver(FrameRing<kCapacity>& ring, std::size_t bytesPerFrame)
        : ring_{ring}, bytesPerFrame_{bytesPerFrame} {}

    std::size_t write(std::size_t frames) {
        const Regions regions = ring_.acquireWrite(frames);
        fill(regions.region1());
        fill(regions.region2());
        REQUIRE(ring_.commitWrite(regions.frames()));
        return regions.frames();
    }

    std::size_t readAndVerify(std::size_t frames) {
        const Regions regions = ring_.acquireRead(frames);
        verify(regions.region1());
        verify(regions.region2());
        REQUIRE(ring_.commitRead(regions.frames()));
        return regions.frames();
    }

private:
    void fill(const Region& region) {
        const std::size_t bytes = region.len * bytesPerFrame_;
        for (std::size_t i = 0; i < bytes; ++i) {
            region.buf[i] = patternByte(writePos_ + i);
        }
        writePos_ += bytes;
    }

    void verify(const Region& region) {
        const std::size_t bytes = region.len * bytesPerFrame_;
        bool matched = true;
        std::size_t badAt = 0;
        int got = 0;
        int want = 0;
        for (std::size_t i = 0; i < bytes; ++i) {
            const std::uint8_t expected = patternByte(readPos_ + i);
            if (region.buf[i] != expected) {
                matched = false;
                badAt = readPos_ + i;
                got = static_cast<int>(region.buf[i]);
                want = static_cast<int>(expected);
                break;
            }
        }
        readPos_ += bytes;
        INFO("first mismatch at stream byte " << badAt << ": got " << got << ", want " << want);
        REQUIRE(matched);
    }

    FrameRing<kCapacity>& ring_;
    std::size_t bytesPerFrame_;
    std::size_t writePos_ = 0;
    std::size_t readPos_ = 0;
};

TEST_CASE("init accepts only valid frame sizes", "[frame_ring]") {
    struct Case { std::size_t bytesPerFrame; bool accepted; };
    constexpr Case cases[] = {
        { 0, false}, { 1, false}, { 2, true }, { 4, true }, { 6, true },
        { 8, true }, {16, true }, {17, false}, {32, false},
    };

    for (const Case c : cases) {
        auto ring = makeRing();
        CAPTURE(c.bytesPerFrame);
        CHECK(ring->init(c.bytesPerFrame) == c.accepted);
    }
}

TEST_CASE("correct available values for empty ring", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(2));
    REQUIRE(ring->availableRead() == 0);
    REQUIRE(ring->availableWrite() == FrameRing<kCapacity>::capacity());
}

TEST_CASE("init can only succeed once", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(4));
    REQUIRE_FALSE(ring->init(8));       // different value
    REQUIRE_FALSE(ring->init(4));       // same value
}

TEST_CASE("a rejected init can be retried", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE_FALSE(ring->init(0));
    REQUIRE(ring->init(4));
}

TEST_CASE("single region in case there is no wrap around", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(2));
    const Regions regions = ring->acquireWrite(8);
    REQUIRE(regions.frames() == 8);
    REQUIRE(regions.region1().len == 8);
    REQUIRE(regions.region1().buf != nullptr);
    REQUIRE(regions.region2().len == 0);
    REQUIRE(regions.region2().buf == nullptr);
    REQUIRE(ring->commitWrite(regions.frames()));
    REQUIRE(ring->availableWrite() == 8);
    REQUIRE(ring->availableRead() == 8);
}

TEST_CASE("a write is clamped to the free space and yields nothing when the ring is full",
          "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(2));
    REQUIRE(ring->availableWrite() == ring->capacity());
    {
        const Regions regions = ring->acquireWrite(10);
        REQUIRE(regions.frames() == 10);
        REQUIRE(regions.region1().len == 10);
        REQUIRE(regions.region1().buf != nullptr);
        REQUIRE(regions.region2().len == 0);
        REQUIRE(regions.region2().buf == nullptr);
        REQUIRE(ring->commitWrite(10));
        REQUIRE(ring->availableWrite() == 6);
        REQUIRE(ring->availableRead() == 10);
    }
    {
        const Regions regions = ring->acquireWrite(10);
        REQUIRE(regions.frames() == 6);
        REQUIRE(regions.region1().len == 6);
        REQUIRE(regions.region1().buf != nullptr);
        REQUIRE(regions.region2().len == 0);
        REQUIRE(regions.region2().buf == nullptr);
        REQUIRE(ring->commitWrite(6));
        REQUIRE(ring->availableWrite() == 0);
        REQUIRE(ring->availableRead() == ring->capacity());
    }
    {
        REQUIRE(ring->availableWrite() == 0);
        const Regions regions = ring->acquireWrite(10);
        REQUIRE(regions.frames() == 0);
        REQUIRE(regions.region1().len == 0);
        REQUIRE(regions.region1().buf == nullptr);
        REQUIRE(regions.region2().len == 0);
        REQUIRE(regions.region2().buf == nullptr);
    }
}

TEST_CASE("reading back everything written leaves the ring empty", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(2));
    {
        const Regions regions = ring->acquireWrite(8);
        REQUIRE(regions.frames() == 8);
        REQUIRE(regions.region1().len == 8);
        REQUIRE(regions.region1().buf != nullptr);
        REQUIRE(regions.region2().len == 0);
        REQUIRE(regions.region2().buf == nullptr);
        REQUIRE(ring->commitWrite(regions.frames()));
        REQUIRE(ring->availableWrite() == 8);
        REQUIRE(ring->availableRead() == 8);
    }
    {
        const Regions regions = ring->acquireRead(8);
        REQUIRE(regions.frames() == 8);
        REQUIRE(regions.region1().len == 8);
        REQUIRE(regions.region1().buf != nullptr);
        REQUIRE(regions.region2().len == 0);
        REQUIRE(regions.region2().buf == nullptr);
        REQUIRE(ring->commitRead(regions.frames()));
        REQUIRE(ring->availableWrite() == ring->capacity());
        REQUIRE(ring->availableRead() == 0);
    }
}

TEST_CASE("a write that runs past the end of the buffer splits into two regions", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(2));
    {
        const Regions regions = ring->acquireWrite(12);
        REQUIRE(regions.frames() == 12);
        REQUIRE(regions.region1().len == 12);
        REQUIRE(regions.region1().buf != nullptr);
        REQUIRE(regions.region2().len == 0);
        REQUIRE(regions.region2().buf == nullptr);
        REQUIRE(ring->commitWrite(regions.frames()));
        REQUIRE(ring->availableWrite() == 4);
        REQUIRE(ring->availableRead() == 12);
    }
    {
        const Regions regions = ring->acquireRead(4);
        REQUIRE(regions.frames() == 4);
        REQUIRE(regions.region1().len == 4);
        REQUIRE(regions.region1().buf != nullptr);
        REQUIRE(regions.region2().len == 0);
        REQUIRE(regions.region2().buf == nullptr);
        REQUIRE(ring->commitRead(regions.frames()));
        REQUIRE(ring->availableWrite() == 8);
        REQUIRE(ring->availableRead() == 8);
    }
    {
        const Regions regions = ring->acquireWrite(8);
        REQUIRE(regions.frames() == 8);
        REQUIRE(regions.region1().len == 4);
        REQUIRE(regions.region1().buf != nullptr);
        REQUIRE(regions.region2().len == 4);
        REQUIRE(regions.region2().buf != nullptr);
        REQUIRE(ring->commitWrite(regions.frames()));
        REQUIRE(ring->availableWrite() == 0);
        REQUIRE(ring->availableRead() == ring->capacity());
    }
}

TEST_CASE("data survives a write and a read without wrapping", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(4));
    PatternDriver driver{*ring, 4};

    REQUIRE(driver.write(8) == 8);
    REQUIRE(driver.readAndVerify(8) == 8);
    REQUIRE(ring->availableRead() == 0);
}

TEST_CASE("data survives the wrap", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(4));
    PatternDriver driver{*ring, 4};

    REQUIRE(driver.write(12) == 12);
    REQUIRE(driver.readAndVerify(4) == 4);
    REQUIRE(driver.write(8) == 8);
    REQUIRE(ring->availableRead() == ring->capacity());
    REQUIRE(driver.readAndVerify(16) == 16);
}

TEST_CASE("data survives many laps at several frame sizes", "[frame_ring]") {
    constexpr std::size_t frameSizes[] = {2, 4, 6};

    for (const std::size_t bytesPerFrame : frameSizes) {
        CAPTURE(bytesPerFrame);
        auto ring = makeRing();
        REQUIRE(ring->init(bytesPerFrame));
        PatternDriver driver{*ring, bytesPerFrame};

        REQUIRE(driver.write(3) == 3);
        for (int lap = 0; lap < 20; ++lap) {
            CAPTURE(lap);
            REQUIRE(driver.write(7) == 7);
            REQUIRE(driver.readAndVerify(7) == 7);
        }
        REQUIRE(driver.readAndVerify(3) == 3);
        REQUIRE(ring->availableRead() == 0);
    }
}

// `frames` frames of the pattern, as it would appear starting at `streamPos`.
std::vector<std::uint8_t> makePattern(std::size_t frames, std::size_t bytesPerFrame, std::size_t streamPos) {
    std::vector<std::uint8_t> out(frames * bytesPerFrame);
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = patternByte(streamPos + i);
    }
    return out;
}

TEST_CASE("the copy helpers round-trip data across the wrap", "[frame_ring]") {
    constexpr std::size_t kBytesPerFrame = 4;
    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    const std::vector<std::uint8_t> first = makePattern(12, kBytesPerFrame, 0);
    REQUIRE(ring->write(first.data(), 12));
    std::vector<std::uint8_t> back(first.size());
    REQUIRE(ring->read(back.data(), 12) == 12);
    REQUIRE(back == first);

    const std::vector<std::uint8_t> second = makePattern(8, kBytesPerFrame, first.size());
    REQUIRE(ring->write(second.data(), 8));
    std::vector<std::uint8_t> back2(second.size());
    REQUIRE(ring->read(back2.data(), 8) == 8);
    REQUIRE(back2 == second);
    REQUIRE(ring->availableRead() == 0);
}

TEST_CASE("write takes all the frames or none of them", "[frame_ring]") {
    constexpr std::size_t kBytesPerFrame = 2;
    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    const std::vector<std::uint8_t> tooMany = makePattern(kCapacity + 1, kBytesPerFrame, 0);
    REQUIRE_FALSE(ring->write(tooMany.data(), kCapacity + 1));
    REQUIRE(ring->availableRead() == 0);
    REQUIRE(ring->availableWrite() == ring->capacity());

    const std::vector<std::uint8_t> exact = makePattern(kCapacity, kBytesPerFrame, 0);
    REQUIRE(ring->write(exact.data(), kCapacity));
    REQUIRE(ring->availableWrite() == 0);
}

TEST_CASE("read clamps to what is available and leaves the rest alone", "[frame_ring]") {
    constexpr std::size_t kBytesPerFrame = 2;
    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    const std::vector<std::uint8_t> five = makePattern(5, kBytesPerFrame, 0);
    REQUIRE(ring->write(five.data(), 5));

    constexpr std::uint8_t kFill = 0xEE;
    std::vector<std::uint8_t> out(10 * kBytesPerFrame, kFill);
    REQUIRE(ring->read(out.data(), 10) == 5);
    REQUIRE(std::equal(five.begin(), five.end(), out.begin()));
    REQUIRE(out[five.size()] == kFill);
    REQUIRE(ring->availableRead() == 0);

    REQUIRE(ring->read(out.data(), 10) == 0);
}

TEST_CASE("zero frames is a no-op that succeeds", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(2));

    std::uint8_t unused = 0;
    REQUIRE(ring->write(&unused, 0));
    REQUIRE(ring->availableRead() == 0);
    REQUIRE(ring->read(&unused, 0) == 0);
    REQUIRE(ring->availableWrite() == ring->capacity());
}

TEST_CASE("a write commit advances by what was committed, not what was acquired", "[frame_ring]") {
    constexpr std::size_t kBytesPerFrame = 4;
    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    const Regions acquired = ring->acquireWrite(8);
    REQUIRE(acquired.frames() == 8);
    REQUIRE(ring->commitWrite(3));
    REQUIRE(ring->availableRead() == 3);
    REQUIRE(ring->availableWrite() == ring->capacity() - 3);

    const Regions next = ring->acquireWrite(2);
    REQUIRE(next.region1().buf == acquired.region1().buf + 3 * kBytesPerFrame);
}

TEST_CASE("a read commit advances by what was committed, not what was acquired", "[frame_ring]") {
    constexpr std::size_t kBytesPerFrame = 4;
    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    const std::vector<std::uint8_t> eight = makePattern(8, kBytesPerFrame, 0);
    REQUIRE(ring->write(eight.data(), 8));

    const Regions acquired = ring->acquireRead(8);
    REQUIRE(acquired.frames() == 8);
    REQUIRE(ring->commitRead(3));
    REQUIRE(ring->availableRead() == 5);

    std::vector<std::uint8_t> rest(5 * kBytesPerFrame);
    REQUIRE(ring->read(rest.data(), 5) == 5);
    REQUIRE(rest == makePattern(5, kBytesPerFrame, 3 * kBytesPerFrame));
}

TEST_CASE("a rejected commit leaves the counters untouched", "[frame_ring]") {
    constexpr std::size_t kBytesPerFrame = 2;
    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    REQUIRE_FALSE(ring->commitWrite(kCapacity + 1));
    REQUIRE(ring->availableRead() == 0);
    REQUIRE(ring->availableWrite() == ring->capacity());

    REQUIRE_FALSE(ring->commitRead(1));
    REQUIRE(ring->availableRead() == 0);
    REQUIRE(ring->availableWrite() == ring->capacity());

    const std::vector<std::uint8_t> four = makePattern(4, kBytesPerFrame, 0);
    REQUIRE(ring->write(four.data(), 4));
    REQUIRE_FALSE(ring->commitRead(5));
    REQUIRE(ring->availableRead() == 4);
    REQUIRE(ring->availableWrite() == ring->capacity() - 4);
}

TEST_CASE("the second region starts at the beginning of the buffer", "[frame_ring]") {
    constexpr std::size_t kBytesPerFrame = 4;
    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    const std::vector<std::uint8_t> filler = makePattern(12, kBytesPerFrame, 0);
    REQUIRE(ring->write(filler.data(), 12));
    std::vector<std::uint8_t> drained(filler.size());
    REQUIRE(ring->read(drained.data(), 12) == 12);

    const Regions regions = ring->acquireWrite(8);
    REQUIRE(regions.region1().len == 4);
    REQUIRE(regions.region2().len == 4);
    REQUIRE(regions.region2().buf < regions.region1().buf);

    REQUIRE(regions.region1().buf - regions.region2().buf == static_cast<std::ptrdiff_t>(12 * kBytesPerFrame));
}

TEST_CASE("a producer and a consumer thread see every byte in order", "[frame_ring][threads]") {
    constexpr std::size_t kBytesPerFrame = 4;
    constexpr std::size_t kTotalFrames = 100000;
    constexpr std::size_t kWriteChunk = 5;
    constexpr std::size_t kReadChunk = 7;

    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    std::atomic<bool> stop{false};

    std::thread producer([&] {
        std::vector<std::uint8_t> chunk(kWriteChunk * kBytesPerFrame);
        std::size_t streamPos = 0;
        std::size_t written   = 0;
        while (written < kTotalFrames) {
            const std::size_t frames = std::min(kWriteChunk, kTotalFrames - written);
            for (std::size_t i = 0; i < frames * kBytesPerFrame; ++i) {
                chunk[i] = patternByte(streamPos + i);
            }
            while (!ring->write(chunk.data(), frames)) {
                if (stop.load(std::memory_order_relaxed)) return;
                std::this_thread::yield();
            }
            streamPos += frames * kBytesPerFrame;
            written += frames;
        }
    });

    std::vector<std::uint8_t> out(kReadChunk * kBytesPerFrame);
    std::size_t readPos = 0;
    std::size_t framesRead = 0;
    bool matched = true;
    std::size_t badAt = 0;
    int got = 0;
    int want = 0;

    while (framesRead < kTotalFrames && matched) {
        const std::size_t frames = ring->read(out.data(), kReadChunk);
        if (frames == 0) {
            std::this_thread::yield();
            continue;
        }
        for (std::size_t i = 0; i < frames * kBytesPerFrame; ++i) {
            const std::uint8_t expected = patternByte(readPos + i);
            if (out[i] != expected) {
                matched = false;
                badAt = readPos + i;
                got = static_cast<int>(out[i]);
                want = static_cast<int>(expected);
                break;
            }
        }
        readPos += frames * kBytesPerFrame;
        framesRead += frames;
    }

    stop.store(true, std::memory_order_relaxed);
    producer.join();

    INFO("first mismatch at stream byte " << badAt << ": got " << got << ", want " << want);
    REQUIRE(matched);
    REQUIRE(framesRead == kTotalFrames);
    REQUIRE(ring->availableRead() == 0);
}

}
