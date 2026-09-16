#include "servo.h"
#include <algorithm>
#include <cmath>
#include <numbers>

Servo::Servo() : state_{}, acquireStart_{0.0} {

}

void Servo::configure(double acquireBandwidthHz, double lockBandwidthHz) {
    acquireBw_ = acquireBandwidthHz;
    lockBw_    = lockBandwidthHz;
}

void Servo::reset() {
    current_.reset();
    state_ = State{};
    acquireStart_ = 0.0;
    seedCount_ = 0;
    windowMin_ = std::chrono::nanoseconds::max();
    pathDelay_ = std::chrono::nanoseconds{0};
    windowStart_ = 0.0;
    accepted_ = 0;
    rejected_ = 0;
    tooSoon_ = 0;
    consecutiveLarge_ = 0;
}

void Servo::addSample(double localSeconds, double masterSeconds,
                      std::chrono::nanoseconds delay) {
    // 1. window first: the gate below depends on it
    if (delay < windowMin_) windowMin_ = delay;
    if (pathDelay_.count() == 0 || delay < pathDelay_) pathDelay_ = delay;
    if (localSeconds - windowStart_ >= kWindowSec) {
        pathDelay_   = windowMin_;
        windowMin_   = std::chrono::nanoseconds::max();
        windowStart_ = localSeconds;
    }

    // 2. seeding
    if (!current_) {
        seeds_[seedCount_++] = {localSeconds, masterSeconds, delay};
        if (seedCount_ < seeds_.size()) return;
        const auto& best = *std::min_element(seeds_.begin(), seeds_.end(),
            [](const Seed& x, const Seed& y) { return x.delay < y.delay; });
        current_ = ClockMapping{.localRef = best.L, .masterRef = best.M, .skew = 0.0};
        acquireStart_ = localSeconds;
        windowStart_  = localSeconds;
        state_.value  = State::Acquiring;
        return;
    }

    // 3. now the gate is meaningful
    if (pathDelay_.count() > 0 && delay > 2 * pathDelay_) { ++rejected_; return; }

    const double T = localSeconds - current_->localRef;
    if (T < 0.01) {
        ++tooSoon_;
        return;
    }

    const double elapsed = localSeconds - acquireStart_;
    const double bw = lockBw_ + (acquireBw_ - lockBw_) * std::exp(-elapsed / 2.0);
    double w = 2.0 * std::numbers::pi * bw * T;
    if (w > 0.4) w = 0.4;
    const double b = std::numbers::sqrt2 * w;
    const double cOverT = w * w / T;

    const double predicted = localToMaster(localSeconds, *current_);
    const double e = masterSeconds - predicted;

    if (std::abs(e) > 1e-3) {
        if (++consecutiveLarge_ >= 5) { reset(); return; }
    } else {
        consecutiveLarge_ = 0;
    }

    current_->localRef  = localSeconds;
    current_->masterRef = predicted + b * e;
    current_->skew     += cOverT * e;

    ++accepted_;
    if (state_.value == State::Acquiring &&
        localSeconds - acquireStart_ > 8.0 && accepted_ >= 32) {
        state_.value = State::Locked;
    }
}

std::optional<ClockMapping> Servo::mapping() const {
    return current_;
}

State Servo::state() const {
    return state_;
}

std::chrono::nanoseconds Servo::pathDelay() const { return pathDelay_; }
std::uint64_t Servo::rejected() const { return rejected_; }
std::uint64_t Servo::tooSoon() const { return tooSoon_; }

[[nodiscard]] double localToMaster(double local, const ClockMapping& mapping) noexcept {
    return mapping.masterRef + (local - mapping.localRef) * (1.0 + mapping.skew);
}

[[nodiscard]] double masterToLocal(double master, const ClockMapping& mapping) noexcept {
    return mapping.localRef + (master - mapping.masterRef) / (1.0 + mapping.skew);
}

