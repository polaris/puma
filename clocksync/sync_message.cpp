#include "sync_message.h"

#include "byte_order.h"

namespace clocksync {
namespace {

constexpr std::size_t kOffMagic    =  0;
constexpr std::size_t kOffVersion  =  4;
constexpr std::size_t kOffType     =  5;
constexpr std::size_t kOffDomain   =  6;
constexpr std::size_t kOffFlags    =  7;
constexpr std::size_t kOffNodeId   =  8;
constexpr std::size_t kOffTargetId = 16;
constexpr std::size_t kOffSeq      = 24;
constexpr std::size_t kOffRefSeq   = 28;
constexpr std::size_t kOffT        = 32;

static_assert(kOffT + 8 == kMessageBytes, "layout and kMessageBytes disagree");

bool knownType(std::uint8_t v) noexcept {
    return v == static_cast<std::uint8_t>(MsgType::Sync) ||
           v == static_cast<std::uint8_t>(MsgType::DelayReq) ||
           v == static_cast<std::uint8_t>(MsgType::DelayResp);
}

}  // namespace

void encode(const SyncMessage& msg, std::uint8_t (&out)[kMessageBytes]) noexcept {
    put_u32(out + kOffMagic, kMagic);
    out[kOffVersion] = msg.version;
    out[kOffType]    = static_cast<std::uint8_t>(msg.type);
    out[kOffDomain]  = msg.domain;
    out[kOffFlags]   = msg.flags;
    put_u64(out + kOffNodeId,   msg.nodeId);
    put_u64(out + kOffTargetId, msg.targetId);
    put_u32(out + kOffSeq,      msg.seq);
    put_u32(out + kOffRefSeq,   msg.refSeq);
    put_u64(out + kOffT,        msg.t);
}

std::optional<SyncMessage> decode(const std::uint8_t* data,
                                  std::size_t size) noexcept {
    // Size first: every access below assumes the full message is present.
    if (data == nullptr || size != kMessageBytes) return std::nullopt;
    if (get_u32(data + kOffMagic) != kMagic)      return std::nullopt;

    const std::uint8_t version = data[kOffVersion];
    if (version != kVersion) return std::nullopt;

    const std::uint8_t type = data[kOffType];
    if (!knownType(type)) return std::nullopt;

    // flags is deliberately not validated: unknown bits must be ignored so that
    // a future sender setting twoStep is not rejected by an older receiver.

    SyncMessage msg;
    msg.version  = version;
    msg.type     = static_cast<MsgType>(type);
    msg.domain   = data[kOffDomain];
    msg.flags    = data[kOffFlags];
    msg.nodeId   = get_u64(data + kOffNodeId);
    msg.targetId = get_u64(data + kOffTargetId);
    msg.seq      = get_u32(data + kOffSeq);
    msg.refSeq   = get_u32(data + kOffRefSeq);
    msg.t        = get_u64(data + kOffT);
    return msg;
}

}  // namespace clocksync
