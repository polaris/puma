#ifndef CLOCK_HELPER_H
#define CLOCK_HELPER_H

#include <chrono>
#include <cstdint>

using Clock = std::chrono::steady_clock;

inline std::uint64_t nowNanos() noexcept {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count());
}

inline std::uint64_t toNanos(Clock::time_point tp) noexcept {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            tp.time_since_epoch()).count());
}

constexpr double toSeconds(std::chrono::nanoseconds d) noexcept {
    return std::chrono::duration<double>(d).count();
}


#endif // CLOCK_HELPER_H