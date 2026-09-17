#include "sync_message.h"

#include "byte_order.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>

using clocksync::decode;
using clocksync::encode;
using clocksync::kMagic;
using clocksync::kMessageBytes;
using clocksync::kVersion;
using clocksync::MsgType;
using clocksync::SyncMessage;

namespace {

/// Every field set to something distinctive, so a swapped offset shows up.
SyncMessage sample(MsgType type) {
    SyncMessage msg;
    msg.version  = kVersion;
    msg.type     = type;
    msg.domain   = 7;
    msg.flags    = 0x01;
    msg.nodeId   = 0x0123456789ABCDEFull;
    msg.targetId = 0xFEDCBA9876543210ull;
    msg.seq      = 0x11223344u;
    msg.refSeq   = 0x55667788u;
    msg.t        = 0x00000123456789ABull;
    return msg;
}

}  // namespace

TEST_CASE("encode/decode round-trips every message type", "[wire][sync_message]") {
    const MsgType types[] = {MsgType::Sync, MsgType::DelayReq, MsgType::DelayResp};

    for (MsgType type : types) {
        const SyncMessage original = sample(type);

        std::uint8_t buf[kMessageBytes]{};
        encode(original, buf);

        const auto parsed = decode(buf, sizeof buf);
        REQUIRE(parsed.has_value());
        REQUIRE(*parsed == original);
    }
}

TEST_CASE("encode writes the documented wire layout", "[wire][sync_message]") {
    const SyncMessage msg = sample(MsgType::DelayResp);

    std::uint8_t buf[kMessageBytes]{};
    encode(msg, buf);

    REQUIRE(get_u32(buf + 0) == kMagic);
    REQUIRE(buf[0] == 'P');
    REQUIRE(buf[1] == 'U');
    REQUIRE(buf[2] == 'M');
    REQUIRE(buf[3] == 'A');
    REQUIRE(buf[4] == kVersion);
    REQUIRE(buf[5] == static_cast<std::uint8_t>(MsgType::DelayResp));
    REQUIRE(buf[6] == msg.domain);
    REQUIRE(buf[7] == msg.flags);
    REQUIRE(get_u64(buf + 8) == msg.nodeId);
    REQUIRE(get_u64(buf + 16) == msg.targetId);
    REQUIRE(get_u32(buf + 24) == msg.seq);
    REQUIRE(get_u32(buf + 28) == msg.refSeq);
    REQUIRE(get_u64(buf + 32) == msg.t);
}

TEST_CASE("decode rejects malformed datagrams", "[wire][sync_message]") {
    std::uint8_t buf[kMessageBytes]{};
    encode(sample(MsgType::Sync), buf);

    SECTION("null data") {
        REQUIRE_FALSE(decode(nullptr, kMessageBytes).has_value());
    }

    SECTION("wrong size") {
        REQUIRE_FALSE(decode(buf, 0).has_value());
        REQUIRE_FALSE(decode(buf, kMessageBytes - 1).has_value());
        REQUIRE_FALSE(decode(buf, kMessageBytes + 1).has_value());
    }

    SECTION("wrong magic") {
        buf[0] ^= 0xFF;
        REQUIRE_FALSE(decode(buf, kMessageBytes).has_value());
    }

    SECTION("wrong version") {
        buf[4] = kVersion + 1;
        REQUIRE_FALSE(decode(buf, kMessageBytes).has_value());
        buf[4] = 0;
        REQUIRE_FALSE(decode(buf, kMessageBytes).has_value());
    }

    SECTION("unknown type") {
        for (std::uint8_t type : {std::uint8_t{0}, std::uint8_t{4}, std::uint8_t{255}}) {
            buf[5] = type;
            REQUIRE_FALSE(decode(buf, kMessageBytes).has_value());
        }
    }
}

TEST_CASE("decode ignores unknown flag bits", "[wire][sync_message]") {
    // Forward compatibility: a future sender setting a flag we do not know must
    // not be rejected, and the bits must survive so a relay can pass them on.
    SyncMessage msg = sample(MsgType::Sync);
    msg.flags = 0xFF;

    std::uint8_t buf[kMessageBytes]{};
    encode(msg, buf);

    const auto parsed = decode(buf, sizeof buf);
    REQUIRE(parsed.has_value());
    REQUIRE(parsed->flags == 0xFF);
}

TEST_CASE("decode does not filter by domain or nodeId", "[wire][sync_message]") {
    // Both are the caller's policy, not decode()'s: it only answers whether the
    // datagram is structurally one of ours.
    SyncMessage msg = sample(MsgType::DelayReq);
    msg.domain = 200;
    msg.nodeId = 0;

    std::uint8_t buf[kMessageBytes]{};
    encode(msg, buf);

    const auto parsed = decode(buf, sizeof buf);
    REQUIRE(parsed.has_value());
    REQUIRE(parsed->domain == 200);
    REQUIRE(parsed->nodeId == 0);
}
