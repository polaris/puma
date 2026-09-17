#include "byte_order.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <limits>

namespace {

constexpr std::uint8_t kSentinel = 0xAB;

}  // namespace

TEST_CASE("put_u32/get_u32 round-trip", "[wire][byte_order]") {
    std::array<std::uint8_t, 4> buf{};

    const std::uint32_t values[] = {
        0u, 1u, 0x01234567u, 0x80000000u, std::numeric_limits<std::uint32_t>::max()};

    for (std::uint32_t v : values) {
        put_u32(buf.data(), v);
        REQUIRE(get_u32(buf.data()) == v);
    }
}

TEST_CASE("put_u64/get_u64 round-trip", "[wire][byte_order]") {
    std::array<std::uint8_t, 8> buf{};

    const std::uint64_t values[] = {
        0u, 1u, 0x0123456789ABCDEFull, 0x8000000000000000ull,
        std::numeric_limits<std::uint64_t>::max()};

    for (std::uint64_t v : values) {
        put_u64(buf.data(), v);
        REQUIRE(get_u64(buf.data()) == v);
    }
}

TEST_CASE("the wire encoding is big endian", "[wire][byte_order]") {
    // This is the test that catches a host-endianness regression: a round-trip
    // alone would still pass if both halves flipped together.
    SECTION("32 bit") {
        std::array<std::uint8_t, 4> buf{};
        put_u32(buf.data(), 0x01020304u);
        REQUIRE(buf[0] == 0x01);
        REQUIRE(buf[1] == 0x02);
        REQUIRE(buf[2] == 0x03);
        REQUIRE(buf[3] == 0x04);
    }

    SECTION("64 bit") {
        std::array<std::uint8_t, 8> buf{};
        put_u64(buf.data(), 0x0102030405060708ull);
        for (std::size_t i = 0; i < buf.size(); ++i) {
            REQUIRE(buf[i] == static_cast<std::uint8_t>(i + 1));
        }
    }

    SECTION("get_ reads the same order") {
        const std::uint8_t raw32[] = {0xDE, 0xAD, 0xBE, 0xEF};
        REQUIRE(get_u32(raw32) == 0xDEADBEEFu);

        const std::uint8_t raw64[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
        REQUIRE(get_u64(raw64) == 0xDEADBEEFCAFEBABEull);
    }
}

TEST_CASE("writes touch exactly the field width", "[wire][byte_order]") {
    SECTION("put_u32 writes 4 bytes") {
        std::array<std::uint8_t, 12> buf{};
        buf.fill(kSentinel);
        put_u32(buf.data() + 4, 0xFFFFFFFFu);

        for (std::size_t i = 0; i < 4; ++i) REQUIRE(buf[i] == kSentinel);
        for (std::size_t i = 8; i < buf.size(); ++i) REQUIRE(buf[i] == kSentinel);
    }

    SECTION("put_u64 writes 8 bytes") {
        std::array<std::uint8_t, 16> buf{};
        buf.fill(kSentinel);
        put_u64(buf.data() + 4, 0xFFFFFFFFFFFFFFFFull);

        for (std::size_t i = 0; i < 4; ++i) REQUIRE(buf[i] == kSentinel);
        for (std::size_t i = 12; i < buf.size(); ++i) REQUIRE(buf[i] == kSentinel);
    }
}
