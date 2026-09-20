#include "servo.h"
#include <algorithm>
#include <cmath>
#include <numbers>

void Servo::configure(double acquireBandwidthHz, double lockBandwidthHz) {
    acquireBw_ = acquireBandwidthHz;
    lockBw_    = lockBandwidthHz;
}

void Servo::reset() {
    current_.reset();
    state_ = State{};
    acquireStart_ = 0.0;
    seedCount_ = 0;
    legA_.reset();
    legB_.reset();
    pathDelay_  = std::chrono::nanoseconds{0};
    gateThreshold_ = std::chrono::nanoseconds::max();
    accepted_ = 0;
    rejected_ = 0;
    tooSoon_ = 0;
    consecutiveLarge_ = 0;
}

void Servo::Window::add(std::chrono::nanoseconds v) {
    v_[next_] = v;
    next_ = (next_ + 1) % kSize;
    if (count_ < kSize) ++count_;

    std::array<std::chrono::nanoseconds, kSize> sorted{};
    std::copy_n(v_.begin(), count_, sorted.begin());
    std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(count_));

    floor_ = sorted[0];
    const auto median = sorted[count_ / 2];
    threshold_ = floor_ + std::chrono::nanoseconds{static_cast<std::int64_t>(
        kGateK * static_cast<double>((median - floor_).count()))};
}

void Servo::Window::reset() {
    v_.fill(std::chrono::nanoseconds{0});
    count_ = 0;
    next_  = 0;
    floor_ = std::chrono::nanoseconds{0};
    threshold_ = std::chrono::nanoseconds::max();
}

void Servo::addSample(double localSeconds, double masterSeconds,
                      std::chrono::nanoseconds a, std::chrono::nanoseconds b) {
    const auto delay = (a + b) / 2;   // seeding and reporting still use the mean
    legA_.add(a);
    legB_.add(b);
    // 1. window first: the gate below depends on it. Rejected samples still
    //    inform the window, otherwise a period of high delay would never be
    //    reflected in the threshold and the gate would reject everything.
    // Keep pathDelay()/gateThreshold() meaningful for Stats and for sizing the
    // audio buffer by combining the two legs.
    pathDelay_     = (legA_.floor() + legB_.floor()) / 2;
    gateThreshold_ = (legA_.threshold() == std::chrono::nanoseconds::max() ||
                      legB_.threshold() == std::chrono::nanoseconds::max())
                   ? std::chrono::nanoseconds::max()
                   : (legA_.threshold() + legB_.threshold()) / 2;

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
    // Gate each direction against its own floor. A sample where one leg was
    // queued and the other was not is exactly the asymmetric case, and it
    // passes a gate on the average; this catches it.
    if (legA_.active() && legB_.active() &&
        (a > legA_.threshold() || b > legB_.threshold())) {
        ++rejected_;
        return;
    }

    const double T = localSeconds - current_->localRef;
    if (T < 0.01) {
        ++tooSoon_;
        return;
    }

    const double elapsed = localSeconds - acquireStart_;
    const double bw = lockBw_ + (acquireBw_ - lockBw_) * std::exp(-elapsed / 2.0);
    const double w_n = 2.0 * std::numbers::pi * bw;
    double w = w_n * T;
    if (w > 0.4) w = 0.4;
    const double gainP = std::numbers::sqrt2 * w;   // proportional
    const double gainI = w * w / T;                 // integral, per second

    const double predicted = localToMaster(localSeconds, *current_);
    const double e = masterSeconds - predicted;

    if (std::abs(e) > 1e-3) {
        if (++consecutiveLarge_ >= 5) { reset(); return; }
    } else {
        consecutiveLarge_ = 0;
    }

    current_->localRef  = localSeconds;
    current_->masterRef = predicted + gainP * e;
    current_->skew     += gainI * e;

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
std::chrono::nanoseconds Servo::gateThreshold() const { return gateThreshold_; }
bool Servo::gateActive() const { return legA_.active() && legB_.active(); }
std::chrono::nanoseconds Servo::floorA() const { return legA_.floor(); }
std::chrono::nanoseconds Servo::floorB() const { return legB_.floor(); }
std::uint64_t Servo::rejected() const { return rejected_; }
std::uint64_t Servo::tooSoon() const { return tooSoon_; }

[[nodiscard]] double localToMaster(double local, const ClockMapping& mapping) noexcept {
    return mapping.masterRef + (local - mapping.localRef) * (1.0 + mapping.skew);
}

[[nodiscard]] double masterToLocal(double master, const ClockMapping& mapping) noexcept {
    return mapping.localRef + (master - mapping.masterRef) / (1.0 + mapping.skew);
}