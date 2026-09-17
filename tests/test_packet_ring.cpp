#include "packet_ring.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>

namespace {

/// A PacketRing is ~256 KB (32 slots of 8 KB), so keep it off the stack.
std::unique_ptr<PacketRing> makeRing() { return std::make_unique<PacketRing>(); }

void fill(Slot& slot, std::uint32_t length, std::uint8_t value) {
    slot.length = length;
    for (std::uint32_t i = 0; i < length; ++i) slot.data[i] = value;
}

}  // namespace

TEST_CASE("an empty ring has no front", "[packet_ring]") {
    auto ring = makeRing();

    REQUIRE(ring->front() == nullptr);
    REQUIRE(ring->size() == 0);
}

TEST_CASE("a claimed and committed slot comes back out intact", "[packet_ring]") {
    auto ring = makeRing();

    Slot* slot = ring->claim();
    REQUIRE(slot != nullptr);
    fill(*slot, 128, 0x5A);
    ring->commit();

    REQUIRE(ring->size() == 1);

    const Slot* front = ring->front();
    REQUIRE(front != nullptr);
    REQUIRE(front->length == 128);
    for (std::uint32_t i = 0; i < front->length; ++i) REQUIRE(front->data[i] == 0x5A);

    ring->pop();
    REQUIRE(ring->size() == 0);
    REQUIRE(ring->front() == nullptr);
}

TEST_CASE("the ring is FIFO and tracks its size", "[packet_ring]") {
    auto ring = makeRing();

    constexpr std::uint32_t kCount = 8;
    for (std::uint32_t i = 0; i < kCount; ++i) {
        Slot* slot = ring->claim();
        REQUIRE(slot != nullptr);
        fill(*slot, i + 1, static_cast<std::uint8_t>(i));
        ring->commit();
        REQUIRE(ring->size() == i + 1);
    }

    for (std::uint32_t i = 0; i < kCount; ++i) {
        const Slot* front = ring->front();
        REQUIRE(front != nullptr);
        REQUIRE(front->length == i + 1);
        REQUIRE(front->data[0] == static_cast<std::uint8_t>(i));
        ring->pop();
        REQUIRE(ring->size() == kCount - i - 1);
    }
}

TEST_CASE("the usable capacity is kSlots - 1", "[packet_ring]") {
    // claim() refuses when the next write index would meet the read index, so
    // one slot is always held back to keep full and empty distinguishable.
    auto ring = makeRing();

    for (std::size_t i = 0; i < kSlots - 1; ++i) {
        Slot* slot = ring->claim();
        REQUIRE(slot != nullptr);
        fill(*slot, 4, static_cast<std::uint8_t>(i));
        ring->commit();
    }

    REQUIRE(ring->size() == kSlots - 1);
    REQUIRE(ring->claim() == nullptr);

    // Freeing one slot makes exactly one more claim available.
    ring->pop();
    REQUIRE(ring->claim() != nullptr);
    ring->commit();
    REQUIRE(ring->claim() == nullptr);
}

TEST_CASE("the ring wraps around past kSlots", "[packet_ring]") {
    auto ring = makeRing();

    std::uint8_t next = 0;
    std::uint8_t expected = 0;

    // Several laps at a steady depth of two.
    for (std::size_t lap = 0; lap < kSlots * 3; ++lap) {
        Slot* slot = ring->claim();
        REQUIRE(slot != nullptr);
        fill(*slot, 4, next++);
        ring->commit();

        if (ring->size() > 2) {
            const Slot* front = ring->front();
            REQUIRE(front != nullptr);
            REQUIRE(front->data[0] == expected++);
            ring->pop();
        }
    }

    while (ring->front() != nullptr) {
        REQUIRE(ring->front()->data[0] == expected++);
        ring->pop();
    }
    REQUIRE(expected == next);
    REQUIRE(ring->size() == 0);
}
