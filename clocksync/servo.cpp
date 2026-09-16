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
    delays_.fill(std::chrono::nanoseconds{0});
    delayCount_ = 0;
    delayNext_  = 0;
    pathDelay_  = std::chrono::nanoseconds{0};
    gateThreshold_ = std::chrono::nanoseconds::max();
    accepted_ = 0;
    rejected_ = 0;
    tooSoon_ = 0;
    consecutiveLarge_ = 0;
}

void Servo::addSample(double localSeconds, double masterSeconds,
                      std::chrono::nanoseconds delay) {
    // 1. window first: the gate below depends on it. Rejected samples still
    //    inform the window, otherwise a period of high delay would never be
    //    reflected in the threshold and the gate would reject everything.
    updateDelayWindow(delay);

    // 2. seeding
    if (!current_) {
        seeds_[seedCount_++] = {localSeconds, masterSeconds, delay};
        if (seedCount_ < seeds_.size()) return;
        const auto& best = *std::min_element(seeds_.begin(), seeds_.end(),
            [](const Seed& x, const Seed& y) { return x.delay < y.delay; });
        current_ = ClockMapping{.localRef = best.L, .masterRef = best.M, .skew = 0.0};
        acquireStart_ = localSeconds;
        state_.value  = State::Acquiring;
        return;
    }

    // 3. now the gate is meaningful
    if (delayCount_ >= kMinForGate && delay > gateThreshold_) { ++rejected_; return; }

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

void Servo::updateDelayWindow(std::chrono::nanoseconds delay) {
    delays_[delayNext_] = delay;
    delayNext_ = (delayNext_ + 1) % kDelayWindow;
    if (delayCount_ < kDelayWindow) ++delayCount_;

    std::array<std::chrono::nanoseconds, kDelayWindow> sorted{};
    std::copy_n(delays_.begin(), delayCount_, sorted.begin());
    std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(delayCount_));

    pathDelay_ = sorted[0];
    const auto median = sorted[delayCount_ / 2];
    const auto spread = median - pathDelay_;
    gateThreshold_ = pathDelay_ + std::chrono::nanoseconds{
        static_cast<std::int64_t>(kGateK * static_cast<double>(spread.count()))};
}

std::chrono::nanoseconds Servo::pathDelay() const { return pathDelay_; }
std::chrono::nanoseconds Servo::gateThreshold() const { return gateThreshold_; }
std::uint64_t Servo::rejected() const { return rejected_; }
std::uint64_t Servo::tooSoon() const { return tooSoon_; }

[[nodiscard]] double localToMaster(double local, const ClockMapping& mapping) noexcept {
    return mapping.masterRef + (local - mapping.localRef) * (1.0 + mapping.skew);
}

[[nodiscard]] double masterToLocal(double master, const ClockMapping& mapping) noexcept {
    return mapping.localRef + (master - mapping.masterRef) / (1.0 + mapping.skew);
}