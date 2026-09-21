#include "frame_ring.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <memory>
#include <vector>

namespace {

constexpr std::size_t kCapacity = 16;
std::unique_ptr<FrameRing<kCapacity>> makeRing() { return std::make_unique<FrameRing<kCapacity>>(); }

// The byte that a given absolute position in the stream should carry.
//
// The value has to vary within a frame as well as between frames: if every byte
// of a frame held the same value, a displacement that preserved frame alignment
// - a channel swap, say - would read back as correct. The prime modulus means
// no displacement shorter than 251 bytes can alias into a false match either,
// which a plain truncation to uint8_t would allow every 256 bytes.
std::uint8_t patternByte(std::size_t streamBytePos) {
    return static_cast<std::uint8_t>(streamBytePos % 251);
}

// Drives a ring with that pattern and checks every byte that comes back out.
// It tracks its own stream positions, so a lost, duplicated or displaced byte
// fails at a named offset rather than merely coming out in the wrong order.
class PatternDriver {
public:
    PatternDriver(FrameRing<kCapacity>& ring, std::size_t bytesPerFrame)
        : ring_{ring}, bytesPerFrame_{bytesPerFrame} {}

    // Writes as many of `frames` as fit and commits exactly that many.
    std::size_t write(std::size_t frames) {
        const Regions regions = ring_.acquireWrite(frames);
        fill(regions.region1());
        fill(regions.region2());
        REQUIRE(ring_.commitWrite(regions.frames()));
        return regions.frames();
    }

    // Reads up to `frames`, checking every byte, and commits exactly that many.
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
        bool        matched = true;
        std::size_t badAt   = 0;
        int         got     = 0;
        int         want    = 0;
        for (std::size_t i = 0; i < bytes; ++i) {
            const std::uint8_t expected = patternByte(readPos_ + i);
            if (region.buf[i] != expected) {
                matched = false;
                badAt   = readPos_ + i;
                got     = static_cast<int>(region.buf[i]);
                want    = static_cast<int>(expected);
                break;
            }
        }
        readPos_ += bytes;
        INFO("first mismatch at stream byte " << badAt << ": got " << got << ", want " << want);
        REQUIRE(matched);
    }

    FrameRing<kCapacity>& ring_;
    std::size_t           bytesPerFrame_;
    std::size_t           writePos_ = 0;   // stream byte position of the next byte written
    std::size_t           readPos_  = 0;   // stream byte position of the next byte checked
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

TEST_CASE("two consecutive writes", "[frame_ring]") {
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

TEST_CASE("reading", "[frame_ring]") {
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

TEST_CASE("reading/writing", "[frame_ring]") {
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

    REQUIRE(driver.write(12) == 12);            // slots 0..11
    REQUIRE(driver.readAndVerify(4) == 4);      // frees slots 0..3
    REQUIRE(driver.write(8) == 8);              // slots 12..15 then 0..3: splits
    REQUIRE(ring->availableRead() == ring->capacity());
    REQUIRE(driver.readAndVerify(16) == 16);    // and the read splits too
}

TEST_CASE("data survives many laps at several frame sizes", "[frame_ring]") {
    constexpr std::size_t frameSizes[] = {2, 4, 6};

    for (const std::size_t bytesPerFrame : frameSizes) {
        CAPTURE(bytesPerFrame);
        auto ring = makeRing();
        REQUIRE(ring->init(bytesPerFrame));
        PatternDriver driver{*ring, bytesPerFrame};

        // Prime, so the read and write offsets sit at different phases. Then
        // run laps of 7, which is coprime with the capacity: over 16 laps the
        // write offset visits all 16 slots and produces every split size 1..6.
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
std::vector<std::uint8_t> makePattern(std::size_t frames, std::size_t bytesPerFrame,
                                      std::size_t streamPos) {
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

    // Twelve frames in and straight back out leaves both counters at 12, so the
    // next pair of calls starts at offset 12 and has to split.
    const std::vector<std::uint8_t> first = makePattern(12, kBytesPerFrame, 0);
    REQUIRE(ring->write(first.data(), 12));
    std::vector<std::uint8_t> back(first.size());
    REQUIRE(ring->read(back.data(), 12) == 12);
    REQUIRE(back == first);

    const std::vector<std::uint8_t> second = makePattern(8, kBytesPerFrame, first.size());
    REQUIRE(ring->write(second.data(), 8));        // splits 4 + 4
    std::vector<std::uint8_t> back2(second.size());
    REQUIRE(ring->read(back2.data(), 8) == 8);     // and so does the read
    REQUIRE(back2 == second);
    REQUIRE(ring->availableRead() == 0);
}

TEST_CASE("write takes all the frames or none of them", "[frame_ring]") {
    constexpr std::size_t kBytesPerFrame = 2;
    auto ring = makeRing();
    REQUIRE(ring->init(kBytesPerFrame));

    const std::vector<std::uint8_t> tooMany = makePattern(kCapacity + 1, kBytesPerFrame, 0);
    REQUIRE_FALSE(ring->write(tooMany.data(), kCapacity + 1));
    REQUIRE(ring->availableRead() == 0);                    // ring left untouched
    REQUIRE(ring->availableWrite() == ring->capacity());

    // One frame fewer fits exactly.
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
    REQUIRE(out[five.size()] == kFill);           // the shortfall is the caller's to pad
    REQUIRE(ring->availableRead() == 0);

    REQUIRE(ring->read(out.data(), 10) == 0);     // empty ring yields nothing
}

TEST_CASE("zero frames is a no-op that succeeds", "[frame_ring]") {
    auto ring = makeRing();
    REQUIRE(ring->init(2));

    std::uint8_t unused = 0;
    REQUIRE(ring->write(&unused, 0));             // not a refusal: nothing to do
    REQUIRE(ring->availableRead() == 0);
    REQUIRE(ring->read(&unused, 0) == 0);
    REQUIRE(ring->availableWrite() == ring->capacity());
}

}
