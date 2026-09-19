#ifndef SYNC_MESSAGE_H
#define SYNC_MESSAGE_H

#include <cstddef>
#include <cstdint>
#include <optional>

namespace clocksync {

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

enum class MsgType : std::uint8_t {
    Sync      = 1,  ///< master -> group; t = t1 (send instant)
    DelayReq  = 2,  ///< slave  -> group; t = t3 (send instant)
    DelayResp = 3,  ///< master -> group; t = t4 (arrival of the DelayReq)
};

/// Wire layout, big endian:
///
///   offset  size  field
///      0      4   magic      'PUMA'
///      4      1   version
///      5      1   type
///      6      1   domain
///      7      1   flags      bit0 = twoStep (reserved)
///      8      8   nodeId
///     16      8   targetId   DelayResp: the slave being answered; else 0
///     24      4   seq        per-sender counter
///     28      4   refSeq     DelayResp: echoes the DelayReq's seq; else 0
///     32      8   t          nanoseconds; meaning depends on type
///
inline constexpr std::size_t kMessageBytes = 40;
inline constexpr std::uint32_t kMagic      = 0x50554D41u;  // 'PUMA'
inline constexpr std::uint8_t kVersion     = 1;

struct SyncMessage {
    std::uint8_t  version  = kVersion;
    MsgType       type     = MsgType::Sync;
    std::uint8_t  domain   = 0;
    std::uint8_t  flags    = 0;
    std::uint64_t nodeId   = 0;
    std::uint64_t targetId = 0;
    std::uint32_t seq      = 0;
    std::uint32_t refSeq   = 0;
    std::uint64_t t        = 0;  ///< steady_clock epoch nanoseconds

    friend bool operator==(const SyncMessage&, const SyncMessage&) = default;
};

/// Serialize into exactly kMessageBytes. The array reference lets the compiler
/// check the buffer is large enough.
void encode(const SyncMessage& msg, std::uint8_t (&out)[kMessageBytes]) noexcept;

/// Parse a received datagram. Returns nullopt if it is not a well-formed
/// message of a version and type we understand.
///
/// Deliberately does NOT filter by domain or nodeId: those are policy decisions
/// belonging to the caller, which knows its own identity. decode() only answers
/// "is this a message of ours, structurally".
[[nodiscard]] std::optional<SyncMessage> decode(const std::uint8_t* data,
                                                std::size_t size) noexcept;

}  // namespace clocksync

#endif  // SYNC_MESSAGE_H
