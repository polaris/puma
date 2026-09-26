#include "audio_packet.h"

#include "byte_order.h"

namespace streaming {

namespace {

constexpr std::size_t kAudioPacketOffMagic   =  0;
constexpr std::size_t kAudioPacketOffVersion =  4;
constexpr std::size_t kAudioPacketOffSession =  5;
constexpr std::size_t kAudioPacketOffSeq     =  9;
constexpr std::size_t kAudioPacketOffFrames  = 13;
constexpr std::size_t kAudioPacketOffT       = 17;

static_assert(kAudioPacketOffT + 8 == kAudioPacketHeaderBytes, "layout and kAudioPacketHeaderBytes disagree");

}   // namespace

void encodeAudioPacketHeader(const AudioPacketHeader& header, std::uint8_t (&out)[kAudioPacketHeaderBytes]) noexcept {
    put_u32(out + kAudioPacketOffMagic,    kAudioPacketHeaderMagic);
    out[kAudioPacketOffVersion] = header.version;
    put_u32(out + kAudioPacketOffSession,  header.session);
    put_u32(out + kAudioPacketOffSeq,      header.seq);
    put_u32(out + kAudioPacketOffFrames,   header.frames);
    put_u64(out + kAudioPacketOffT,        header.t);
}

std::optional<AudioPacketHeader> decodeAudioPacketHeader(const std::uint8_t* data, std::size_t size) noexcept {
    if (data == nullptr || size < kAudioPacketHeaderBytes) return std::nullopt;
    if (get_u32(data + kAudioPacketOffMagic) != kAudioPacketHeaderMagic) return std::nullopt;

    const std::uint8_t version = data[kAudioPacketOffVersion];
    if (version != kAudioPacketHeaderVersion) return std::nullopt;

    return AudioPacketHeader{
        .version  = version,
        .session  = get_u32(data + kAudioPacketOffSession),
        .seq      = get_u32(data + kAudioPacketOffSeq),
        .frames   = get_u32(data + kAudioPacketOffFrames),
        .t        = get_u64(data + kAudioPacketOffT),
    };
}

}   // namespace streaming
