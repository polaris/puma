#include "audio_packet.h"

#include "byte_order.h"

#include <catch2/catch_test_macros.hpp>

#include <cstring>

using streaming::AudioPacketHeader;
using streaming::kAudioPacketHeaderVersion;
using streaming::kAudioPacketHeaderBytes;
using streaming::kAudioPacketHeaderMagic;
using streaming::encodeAudioPacketHeader;
using streaming::decodeAudioPacketHeader;

/// Every field set to something distinctive, so a swapped offset shows up.
AudioPacketHeader sample() {
    return AudioPacketHeader{
        .version = kAudioPacketHeaderVersion,
        .session = 0xA1A2A3A4,
        .seq     = 0xB1B2B3B4,
        .frames  = 0xC1C2C3C4,
        .t       = 0x0102030405060708,
    };
}

TEST_CASE("encode/decode round-trip of a header", "[wire][audio_packet]") {
    const AudioPacketHeader original = sample();

    std::uint8_t buf[kAudioPacketHeaderBytes]{};
    encodeAudioPacketHeader(original, buf);

    const auto parsed = decodeAudioPacketHeader(buf, sizeof buf);
    REQUIRE(parsed.has_value());
    REQUIRE(*parsed == original);
}

TEST_CASE("encodeAudioPacketHeader writes the documented wire layout", "[wire][audio_packet]") {
    const AudioPacketHeader header = sample();

    std::uint8_t buf[kAudioPacketHeaderBytes]{};
    encodeAudioPacketHeader(header, buf);

    REQUIRE(get_u32(buf + 0) == kAudioPacketHeaderMagic);
    REQUIRE(buf[0] == 'P');
    REQUIRE(buf[1] == 'A');
    REQUIRE(buf[2] == 'U');
    REQUIRE(buf[3] == 'D');
    REQUIRE(buf[4] == kAudioPacketHeaderVersion);
    REQUIRE(get_u32(buf +  5) == header.session);
    REQUIRE(get_u32(buf +  9) == header.seq);
    REQUIRE(get_u32(buf + 13) == header.frames);
    REQUIRE(get_u64(buf + 17) == header.t);

    for (int i = 0; i < 8; ++i) {
        CAPTURE(i);
        REQUIRE(buf[17 + i] == i + 1);
    }
}

TEST_CASE("decodeAudioPacketHeader rejects malformed datagrams", "[wire][audio_packet]") {
    std::uint8_t buf[kAudioPacketHeaderBytes]{};
    encodeAudioPacketHeader(sample(), buf);

    SECTION("null data") {
        REQUIRE_FALSE(decodeAudioPacketHeader(nullptr, kAudioPacketHeaderBytes).has_value());
    }

    SECTION("wrong size") {
        REQUIRE_FALSE(decodeAudioPacketHeader(buf, 0).has_value());
        REQUIRE_FALSE(decodeAudioPacketHeader(buf, kAudioPacketHeaderBytes - 1).has_value());
    }

    SECTION("wrong magic") {
        buf[0] ^= 0xFF;
        REQUIRE_FALSE(decodeAudioPacketHeader(buf, kAudioPacketHeaderBytes).has_value());
    }

    SECTION("wrong version") {
        buf[4] = kAudioPacketHeaderVersion + 1;
        REQUIRE_FALSE(decodeAudioPacketHeader(buf, kAudioPacketHeaderBytes).has_value());
        buf[4] = 0;
        REQUIRE_FALSE(decodeAudioPacketHeader(buf, kAudioPacketHeaderBytes).has_value());
    }
}

TEST_CASE("decodeAudioPacketHeader accepts a header followed by payload", "[wire][audio_packet]") {
    std::uint8_t header[kAudioPacketHeaderBytes];
    encodeAudioPacketHeader(sample(), header);

    std::uint8_t buf[kAudioPacketHeaderBytes + 16]{};
    std::memcpy(buf, header, sizeof header);

    const auto parsed = decodeAudioPacketHeader(buf, sizeof buf);
    REQUIRE(parsed.has_value());
    REQUIRE(*parsed == sample());
}
