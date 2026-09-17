#include "spsc_ring.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <thread>
#include <vector>

TEST_CASE("an empty ring pops nothing", "[spsc_ring]") {
    SpscRing<int, 4> ring;

    int value = 42;
    REQUIRE_FALSE(ring.pop(value));
    REQUIRE(value == 42);  // the out-param is left alone
}

TEST_CASE("the ring preserves order", "[spsc_ring]") {
    SpscRing<int, 8> ring;

    for (int i = 0; i < 5; ++i) REQUIRE(ring.push(i));

    for (int i = 0; i < 5; ++i) {
        int value = -1;
        REQUIRE(ring.pop(value));
        REQUIRE(value == i);
    }

    int value = -1;
    REQUIRE_FALSE(ring.pop(value));
}

TEST_CASE("the ring holds exactly Capacity elements", "[spsc_ring]") {
    // Unlike PacketRing, the monotonic counters here mean no slot is reserved.
    constexpr std::size_t kCapacity = 4;
    SpscRing<int, kCapacity> ring;

    for (std::size_t i = 0; i < kCapacity; ++i) {
        REQUIRE(ring.push(static_cast<int>(i)));
    }
    REQUIRE_FALSE(ring.push(99));

    int value = -1;
    REQUIRE(ring.pop(value));
    REQUIRE(value == 0);

    // One slot freed, one more push accepted, and no further.
    REQUIRE(ring.push(99));
    REQUIRE_FALSE(ring.push(100));
}

TEST_CASE("the ring wraps around cleanly", "[spsc_ring]") {
    constexpr std::size_t kCapacity = 4;
    SpscRing<int, kCapacity> ring;

    int next = 0;
    int expected = 0;

    // Prime the ring so it stays half full, then run many laps past the
    // capacity one element at a time.
    REQUIRE(ring.push(next++));
    REQUIRE(ring.push(next++));

    for (std::size_t lap = 0; lap < kCapacity * 5; ++lap) {
        REQUIRE(ring.push(next++));

        int value = -1;
        REQUIRE(ring.pop(value));
        REQUIRE(value == expected++);
    }

    while (expected < next) {
        int value = -1;
        REQUIRE(ring.pop(value));
        REQUIRE(value == expected++);
    }
}

TEST_CASE("a producer and a consumer thread see every element in order",
          "[spsc_ring][threads]") {
    constexpr int kCount = 100000;
    SpscRing<int, 1024> ring;

    std::thread producer([&] {
        for (int i = 0; i < kCount; ++i) {
            while (!ring.push(i)) std::this_thread::yield();
        }
    });

    std::vector<int> received;
    received.reserve(kCount);
    for (int i = 0; i < kCount; ++i) {
        int value = -1;
        while (!ring.pop(value)) std::this_thread::yield();
        received.push_back(value);
    }
    producer.join();

    REQUIRE(received.size() == static_cast<std::size_t>(kCount));
    bool inOrder = true;
    for (int i = 0; i < kCount; ++i) {
        if (received[static_cast<std::size_t>(i)] != i) { inOrder = false; break; }
    }
    REQUIRE(inOrder);
}
