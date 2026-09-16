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
    [[nodiscard]] std::chrono::nanoseconds gateThreshold() const;
    [[nodiscard]] std::uint64_t rejected() const;
    [[nodiscard]] std::uint64_t tooSoon() const;

private:
    State state_;
    double acquireBw_;
    double lockBw_;
    double acquireStart_;
    std::optional<ClockMapping> current_;

    // Delay tracking. A rolling window gives both the path-delay estimate (its
    // minimum) and the outlier gate (a threshold relative to the window's
    // spread). A fixed multiple of the minimum does not survive a change in the
    // distribution's shape: switching to kernel timestamps lowered the minimum
    // far more than the median, and a 2x-the-minimum rule then rejected 20% of
    // samples in one direction and 5% in the other on the same pair of hosts.
    static constexpr std::size_t kDelayWindow = 64;   // ~8 s at 8 samples/s
    static constexpr double      kGateK       = 2.0;  // threshold = min + k*(median-min)
    static constexpr std::size_t kMinForGate  = 16;   // don't gate on a tiny window

    std::array<std::chrono::nanoseconds, kDelayWindow> delays_{};
    std::size_t delayCount_ = 0;                      // valid entries, capped
    std::size_t delayNext_  = 0;                      // write cursor
    std::chrono::nanoseconds pathDelay_{0};           // window minimum
    std::chrono::nanoseconds gateThreshold_{std::chrono::nanoseconds::max()};

    void updateDelayWindow(std::chrono::nanoseconds delay);

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