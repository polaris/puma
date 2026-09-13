#ifndef TIME_FILTER_H
#define TIME_FILTER_H

#include <numbers>

#include <miniaudio.h>

class TimeFilter {
public:
    void configure(double bandwidth, ma_uint32 framesPerPeriod, double nominalRate) {
        framesPerPeriod_ = framesPerPeriod;
        nominalPeriod_ = static_cast<double>(framesPerPeriod) / nominalRate;

        const double w = 2.0 * std::numbers::pi * bandwidth * nominalPeriod_;
        b_ = std::numbers::sqrt2 * w;
        c_ = w * w;
    }

    void reset(double t) noexcept {
        t0_ = t;
        e2_ = nominalPeriod_;
        t1_ = t + e2_;
        n0_ = 0;
        n1_ = framesPerPeriod_;
        error_ = 0.0;
        ready_ = true;
    }

    void update(double t) noexcept {
        error_ = t - t1_;
        t0_ = t1_;
        t1_ += b_ * error_ + e2_;
        e2_ += c_ * error_;

        n0_ = n1_;
        n1_ += framesPerPeriod_;
    }

    void skip(std::uint32_t missing) noexcept {
        t1_ += missing * e2_;
        n0_ += missing * framesPerPeriod_;
        n1_ += missing * framesPerPeriod_;
    }

    [[nodiscard]] bool ready() const { return ready_; }
    [[nodiscard]] double time() const { return t0_; }
    [[nodiscard]] std::uint64_t frame() const { return n0_; }
    [[nodiscard]] double period() const { return e2_; }
    [[nodiscard]] double error() const { return error_; }
    [[nodiscard]] double rate() const { return framesPerPeriod_ / e2_; }
    [[nodiscard]] ma_uint32 framesPerPeriod() const { return framesPerPeriod_; }
    void invalidate() { ready_ = false; }

private:
    double b_ = 0.0, c_ = 0.0;
    double nominalPeriod_ = 0.0;
    ma_uint32 framesPerPeriod_ = 0;

    double t0_ = 0.0, t1_ = 0.0, e2_ = 0.0, error_ = 0.0;
    std::uint64_t n0_ = 0, n1_ = 0;
    bool ready_ = false;
};

#endif  // TIME_FILTER_H