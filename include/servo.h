#ifndef SERVO_H
#define SERVO_H

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

struct ClockMapping {
    double localRef;     // a local steady_clock instant, seconds
    double masterRef;    // the master time at that instant
    double skew;         // master seconds per local second, minus 1
};

struct State {
    enum Value { Unsynced, Acquiring, Locked, Holdover } value = Unsynced;
    double holdoverAge = 0;  // seconds since last update
};

class Servo {
public:
    Servo();
    void configure(double acquireBandwidthHz, double lockBandwidthHz);
    void reset();
    void addSample(double localSeconds, double masterSeconds, std::chrono::nanoseconds delay);

    [[nodiscard]] std::optional<ClockMapping> mapping() const;
    [[nodiscard]] State state() const;
    [[nodiscard]] std::chrono::nanoseconds pathDelay() const;
    [[nodiscard]] std::uint64_t rejected() const;
    [[nodiscard]] std::uint64_t tooSoon() const;

private:
    State state_;
    double acquireBw_;
    double lockBw_;
    double acquireStart_;
    std::optional<ClockMapping> current_;

    // delay tracking
    std::chrono::nanoseconds windowMin_{std::chrono::nanoseconds::max()};
    std::chrono::nanoseconds pathDelay_{0};
    double windowStart_ = 0.0;
    static constexpr double kWindowSec = 4.0;

    // seeding
    struct Seed { double L, M; std::chrono::nanoseconds delay; };
    std::array<Seed, 8> seeds_;
    std::size_t seedCount_ = 0;

    // bookkeeping
    std::uint64_t accepted_ = 0, rejected_ = 0, tooSoon_ = 0;
    int consecutiveLarge_ = 0;
};

[[nodiscard]] double localToMaster(double local, const ClockMapping& mapping) noexcept;
[[nodiscard]] double masterToLocal(double master, const ClockMapping& mapping) noexcept;

#endif // SERVO_H
