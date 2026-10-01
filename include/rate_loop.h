#ifndef RATE_LOOP_H
#define RATE_LOOP_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

// One per packet, from the network thread to the audio thread: a point on the
// network side's frame count line. By time `t`, `k` frames will have been
// written to the ring.
struct NetReport {
    double t = 0.0;             // predicted arrival of the next packet (TimeFilter::nextTime())
    std::uint64_t k = 0;        // ring write position once that packet is in
    bool restart = false;       // the network side lost track; earlier reports no longer apply
};

// Steers the resampling ratio so the delay from the network to the device
// stays constant, after Adriaensen, "Controlling adaptive resampling". Runs
// on the audio thread, once per callback; the network thread is the paper's
// ALSA side (it only reports), the callback its Jack side.
//
//   N side: network thread, reports (t_N, k_N) points, k_N = ring write position
//   D side: audio callback, t_D from its TimeFilter, k_D = ring read position
//
// Delay error, the paper's eq. (1) and (2):
//
//   d_N = (k_N1 - k_N0) * (t_D - t_N0) / (t_N1 - t_N0)
//   E   = [k_N0 - k_D] + d_N + d_res - target
//
// E > 0 means more is buffered than wanted: read faster, the ratio goes up.
class RateLoop {
public:
    enum class Phase { Waiting, Settling, Running };

    struct Config {
        double nominalRatio = 1.0;          // sender rate / device rate
        double deviceRate = 48000.0;
        std::size_t framesPerCallback = 0;
        double resamplerDelay = 0.0;        // average d_res while running
        double margin = 0.0;                // frames kept against network jitter
        double bandwidth = 0.05;            // Hz
        double startBandwidth = 0.5;        // Hz, for the first startSeconds after a start
        double startSeconds = 4.0;
        double maxCorrection = 0.005;       // relative, either way
    };

    // What the callback should do.
    struct Step {
        bool play = false;                  // false: output silence and read nothing
        std::uint64_t trim = 0;             // frames to discard from the ring before reading
        double ratio = 1.0;                 // input frames per output frame
    };

    // Starts from scratch, forgetting the learned clock difference too.
    void configure(const Config& config) noexcept {
        config_ = config;
        integral_ = 0.0;
        restart();
    }

    // Back to waiting for the delay to be reached. Keeps the learned clock
    // difference, so a restart settles fast.
    void restart() noexcept {
        phase_ = Phase::Waiting;
        reports_ = 0;
        lowpass1_ = lowpass2_ = 0.0;
    }

    // miniaudio does not promise a fixed callback size.
    void setFramesPerCallback(std::size_t frames) noexcept {
        config_.framesPerCallback = frames;
        setBandwidth(phase_ == Phase::Running ? config_.bandwidth : config_.startBandwidth);
    }

    void addReport(const NetReport& report) noexcept {
        if (report.restart) {
            restart();
        }
        tN0_ = tN1_;
        kN0_ = kN1_;
        tN1_ = report.t;
        kN1_ = report.k;
        ++reports_;
    }

    // Once per callback, before reading. `tD` is the smoothed start time of
    // this callback, `kD` the ring read position, `dRes` the resampler's
    // inputDistance(), `available` what the ring holds now.
    [[nodiscard]] Step update(double tD, std::uint64_t kD, double dRes, std::uint64_t available) noexcept {
        if (reports_ < 2 || !(tN1_ > tN0_)) {
            return {};
        }

        // Integer subtraction first: the counters are too large for a double.
        const double dN = static_cast<double>(kN1_ - kN0_) * (tD - tN0_) / (tN1_ - tN0_);
        error_ = static_cast<double>(static_cast<std::int64_t>(kN0_ - kD)) + dN + dRes - target();

        Step step;
        if (phase_ == Phase::Waiting) {
            // Too little buffered: let the ring fill. The ring has to really
            // hold the target too, bar the packet the smooth line runs ahead
            // by: once the stream stops, the line extrapolates on while the
            // ring stays empty.
            if (error_ < 0.0 || static_cast<double>(available) + packetFrames() < target()) {
                return {};
            }
            // Too much: trim the excess at once (section 3.4), the loop removes the rest.
            step.trim = std::min(static_cast<std::uint64_t>(std::llround(error_)), available);
            error_ -= static_cast<double>(step.trim);
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

    // The delay to keep, in input frames: what one callback reads, the step
    // by which the ring fills (a packet; the N line is smooth), what sits in
    // the resampler, and the margin against jitter.
    [[nodiscard]] double target() const noexcept {
        return config_.nominalRatio * static_cast<double>(config_.framesPerCallback)
             + packetFrames() + config_.resamplerDelay + config_.margin;
    }

    [[nodiscard]] Phase phase() const noexcept { return phase_; }
    [[nodiscard]] double error() const noexcept { return error_; }            // frames, last update
    [[nodiscard]] double correction() const noexcept { return correction_; }  // relative to nominal

private:
    [[nodiscard]] double packetFrames() const noexcept {
        return reports_ >= 2 ? static_cast<double>(kN1_ - kN0_) : 0.0;
    }

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

    double tN0_ = 0.0, tN1_ = 0.0;
    std::uint64_t kN0_ = 0, kN1_ = 0;
    int reports_ = 0;

    double w0_ = 0.0, kp_ = 0.0, ki_ = 0.0;
    double lowpass1_ = 0.0, lowpass2_ = 0.0;
    double integral_ = 0.0;         // learns the clock difference
    double error_ = 0.0;
    double correction_ = 0.0;
};

#endif  // RATE_LOOP_H
