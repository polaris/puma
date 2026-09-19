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
    /// a = t2-t1 (master->slave leg), b = t4-t3 (slave->master leg). Both are
    /// needed rather than just their mean: a sample with one fast and one slow
    /// leg is exactly the asymmetric case, and it passes a gate on the average.
    void addSample(double localSeconds, double masterSeconds,
                   std::chrono::nanoseconds a, std::chrono::nanoseconds b);

    /// Convenience for a caller that only has the round-trip mean: assume the
    /// two legs were equal. That is what a gate on (a+b)/2 implicitly assumed
    /// anyway, so this reproduces the old behaviour exactly -- but it cannot
    /// see path asymmetry, so real callers should pass both legs.
    void addSample(double localSeconds, double masterSeconds,
                   std::chrono::nanoseconds delay) {
        addSample(localSeconds, masterSeconds, delay, delay);
    }

    /// Per-leg floors, for diagnosing path asymmetry: their difference is twice
    /// the systematic offset the two-way exchange cannot detect. Gating does not
    /// remove it -- measured here, keeping only the best third of samples moved
    /// the systematic by under 15% -- because the asymmetry is in the floors,
    /// not in the queueing tail.
    [[nodiscard]] std::chrono::nanoseconds floorA() const;
    [[nodiscard]] std::chrono::nanoseconds floorB() const;

    [[nodiscard]] std::optional<ClockMapping> mapping() const;
    [[nodiscard]] State state() const;
    [[nodiscard]] std::chrono::nanoseconds pathDelay() const;
    [[nodiscard]] std::chrono::nanoseconds gateThreshold() const;

    // A threshold exists from the first sample on, but it is only applied once
    // the window holds kMinForGate entries. Callers reporting the gate should
    // ask this rather than testing gateThreshold() against nanoseconds::max(),
    // which only says that no sample has arrived since the last reset().
    [[nodiscard]] bool gateActive() const;
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
    /// Rolling window over one quantity: its floor (the uncongested value) and
    /// an outlier threshold relative to the window's spread.
    class Window {
    public:
        void add(std::chrono::nanoseconds v);
        void reset();
        [[nodiscard]] std::chrono::nanoseconds floor() const { return floor_; }
        [[nodiscard]] std::chrono::nanoseconds threshold() const { return threshold_; }
        [[nodiscard]] bool active() const { return count_ >= kMinForGate; }
    private:
        static constexpr std::size_t kSize       = 64;   // ~8 s at 8 samples/s
        static constexpr double      kGateK      = 2.0;  // floor + k*(median-floor)
        static constexpr std::size_t kMinForGate = 16;
        std::array<std::chrono::nanoseconds, kSize> v_{};
        std::size_t count_ = 0, next_ = 0;
        std::chrono::nanoseconds floor_{0};
        std::chrono::nanoseconds threshold_{std::chrono::nanoseconds::max()};
    };

    Window legA_;   // master -> slave
    Window legB_;   // slave  -> master
    std::chrono::nanoseconds pathDelay_{0};           // window minimum
    std::chrono::nanoseconds gateThreshold_{std::chrono::nanoseconds::max()};


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