#ifndef RATE_LOOP_H
#define RATE_LOOP_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>

// Steers the resampling ratio so every frame leaves the speaker at its
// presentation time, after Adriaensen, "Controlling adaptive resampling".
// Runs on the audio thread, once per callback.
//
// The error, in input frames, is how far the frame due at the speaker lies
// ahead of the frame about to be played:
//
//   m = localToMaster(t_D + output latency)     when this callback is heard
//   E = [k*(m) - k_D] + d_res                    k*(m) from the PresentationTimeline
//
// E > 0 means late: read faster, the ratio goes up. E < 0 means early.
//
// The loop only steers. Where the error comes from is the caller's business,
// so a target time and a target fill are the same loop.
class RateLoop {
public:
    enum class Phase { Waiting, Settling, Running };

    struct Config {
        double nominalRatio = 1.0;          // sender rate / device rate
        double deviceRate = 48000.0;
        std::size_t framesPerCallback = 0;
        double bandwidth = 0.05;            // Hz
        double startBandwidth = 0.5;        // Hz, for the first startSeconds after a start
        double startSeconds = 4.0;
        double maxCorrection = 0.005;       // relative, either way
        // Past this, in frames, the loop starts over instead of steering: at
        // maxCorrection, steering 5 ms away takes a second.
        double realignFrames = 240.0;
    };

    // What the callback should do.
    struct Step {
        bool play = false;                  // false: output silence and read nothing more
        std::uint64_t trim = 0;             // frames to discard from the ring first, even when not playing
        bool late = false;                  // the trim is frames that were due before they arrived
        double ratio = 1.0;                 // input frames per output frame
    };

    // Starts from scratch, forgetting the learned clock difference too.
    void configure(const Config& config) noexcept {
        config_ = config;
        integral_ = 0.0;
        restart();
    }

    // Back to waiting for the due frame. Keeps the learned clock difference,
    // so a restart settles fast.
    void restart() noexcept {
        phase_ = Phase::Waiting;
        lowpass1_ = lowpass2_ = 0.0;
    }

    // miniaudio does not promise a fixed callback size.
    void setFramesPerCallback(std::size_t frames) noexcept {
        config_.framesPerCallback = frames;
        setBandwidth(phase_ == Phase::Running ? config_.bandwidth : config_.startBandwidth);
    }

    // Once per callback, before reading. `tD` is the smoothed start time of
    // this callback, `error` the error above (nothing while it cannot be
    // known: no clock mapping, too few reports), `available` what the ring
    // holds and `need` what the next read takes at about the current ratio.
    [[nodiscard]] Step update(double tD, std::optional<double> error, std::uint64_t available,
                              std::size_t need) noexcept {
        if (!error) {
            restart();
            return {};
        }
        error_ = *error;

        if (phase_ != Phase::Waiting && std::abs(error_) > config_.realignFrames) {
            restart();
            ++realigns_;
        }

        Step step;
        if (phase_ == Phase::Waiting) {
            // Not due yet: silence until it is.
            if (error_ < 0.0) {
                return {};
            }
            // Due, or past due: the frames before the due one are too late
            // to play (section 3.4: trim at once, the loop removes the rest).
            const auto due = static_cast<std::uint64_t>(std::llround(error_));
            if (due > available) {
                // Not even the due frame is in: all of it is late.
                step.trim = available;
                step.late = true;
                error_ -= static_cast<double>(available);
                return step;
            }
            // The read after the trim must not run dry; the ratio may still
            // move the need by up to maxCorrection.
            const auto slack = static_cast<std::uint64_t>(std::ceil(static_cast<double>(need) * config_.maxCorrection)) + 1;
            if (available - due < need + slack) {
                return {};
            }
            step.trim = due;
            error_ -= static_cast<double>(due);
            phase_ = Phase::Settling;
            settleUntil_ = tD + config_.startSeconds;
            setBandwidth(config_.startBandwidth);
        } else if (phase_ == Phase::Settling && tD >= settleUntil_) {
            phase_ = Phase::Running;
            setBandwidth(config_.bandwidth);
        }

        // Second-order lowpass against high-frequency noise, then PI.
        lowpass1_ += w0_ * (error_ - lowpass1_);
        lowpass2_ += w0_ * (lowpass1_ - lowpass2_);
        integral_ += ki_ * lowpass2_;
        correction_ = std::clamp(kp_ * lowpass2_ + integral_, -config_.maxCorrection, config_.maxCorrection);

        step.play = true;
        step.ratio = config_.nominalRatio * (1.0 + correction_);
        return step;
    }

    [[nodiscard]] Phase phase() const noexcept { return phase_; }
    [[nodiscard]] double error() const noexcept { return error_; }            // frames, last update
    [[nodiscard]] double correction() const noexcept { return correction_; }  // relative to nominal
    [[nodiscard]] std::uint64_t realigns() const noexcept { return realigns_; }

private:
    // Critically damped: both poles at 1 - w, w the loop bandwidth in radians
    // per callback. The gains turn frames of error into a ratio, with one
    // callback reading nominalRatio * framesPerCallback input frames.
    void setBandwidth(double bandwidth) noexcept {
        const auto frames = static_cast<double>(config_.framesPerCallback);
        const double w = 2.0 * std::numbers::pi * bandwidth * frames / config_.deviceRate;
        const double inputPerCallback = config_.nominalRatio * frames;
        w0_ = 1.0 - std::exp(-20.0 * w);     // the lowpass: 20 times the loop bandwidth
        kp_ = 2.0 * w / inputPerCallback;
        ki_ = w * w / inputPerCallback;
    }

    Config config_{};
    Phase phase_ = Phase::Waiting;
    double settleUntil_ = 0.0;

    double w0_ = 0.0, kp_ = 0.0, ki_ = 0.0;
    double lowpass1_ = 0.0, lowpass2_ = 0.0;
    double integral_ = 0.0;         // learns the clock difference
    double error_ = 0.0;
    double correction_ = 0.0;
    std::uint64_t realigns_ = 0;
};

#endif  // RATE_LOOP_H
