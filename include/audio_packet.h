#ifndef AUDIO_PACKET_H
#define AUDIO_PACKET_H

#include <cstddef>
#include <cstdint>
#include <optional>

namespace streaming {

/// Wire layout, big endian:
///
///   offset  size  field
///      0      4   magic      'PAUD'
///      4      1   version    protocol version
///      5      4   session    session ID
///      9      4   seq        per-sender packet counter
///     13      4   frames     number of frames in the packet
///     17      8   t          presentation time of the first frame in ns

inline constexpr std::size_t kAudioPacketHeaderBytes    = 25;
inline constexpr std::uint32_t kAudioPacketHeaderMagic  = 0x50415544u;  // 'PAUD'
inline constexpr std::uint8_t kAudioPacketHeaderVersion = 1;

struct AudioPacketHeader {
    std::uint8_t version    = kAudioPacketHeaderVersion;
    std::uint32_t session   = 0;
    std::uint32_t seq       = 0;
    std::uint32_t frames    = 0;
    std::uint64_t t         = 0;

    friend bool operator==(const AudioPacketHeader&, const AudioPacketHeader&) = default;
};

void encodeAudioPacketHeader(const AudioPacketHeader& header, std::uint8_t (&out)[kAudioPacketHeaderBytes]) noexcept;

[[nodiscard]] std::optional<AudioPacketHeader> decodeAudioPacketHeader(const std::uint8_t* data, std::size_t size) noexcept;

}   // namespace streaming

#endif  // AUDIO_PACKET_H
