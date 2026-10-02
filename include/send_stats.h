#ifndef SEND_STATS_H
#define SEND_STATS_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <asio/error_code.hpp>

// Written on the send thread only, read once it is joined.
struct SendStats {
    std::uint64_t sendErrors = 0;
    std::size_t ringHighWater = 0;      // most packets waiting at one wake-up

    // Details of the most recent occurrence.
    asio::error_code lastSendError;
};

// Written on the audio thread. On cache lines of its own, so the audio
// thread's stores do not contend with anything else.
struct alignas(128) CaptureStats {
    std::atomic<std::uint64_t> droppedPackets{0};   // the ring had no room; the sequence skips them
    std::atomic<std::uint64_t> oversizePackets{0};  // a callback too large for a slot
};
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

#endif  // SEND_STATS_H
