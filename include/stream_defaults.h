#ifndef STREAMING_STREAM_DEFAULTS_H
#define STREAMING_STREAM_DEFAULTS_H

#include <string_view>

// What the sender and the receiver have to agree on, or share by default.
namespace streaming {

constexpr std::string_view kDefaultAudioGroup = "239.255.0.1";
constexpr unsigned short kDefaultAudioPort = 12345;
constexpr std::string_view kClockGroup = "239.255.0.2";    // not the audio group; the sender is the master
constexpr unsigned short kClockPort = 12346;

// Frames per audio callback, and so per packet on the sender, and the
// device's buffer count.
constexpr unsigned int kPeriodSizeInFrames = 240;
constexpr unsigned int kNumPeriods = 3;

// The time filters' smoothing, the sender's capture clock and the receiver's
// device and arrivals alike: fast for the first seconds after a (re)start,
// so start-up bursts settle quickly, then slow.
constexpr double kStartBandwidth = 1.0;    // Hz
constexpr double kBandwidth = 0.05;        // Hz
constexpr double kStartSeconds = 4.0;

}  // namespace streaming

#endif  // STREAMING_STREAM_DEFAULTS_H
