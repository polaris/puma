#ifndef RESAMPLER_H
#define RESAMPLER_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <vector>

// Variable-ratio resampler for interleaved s16 audio. Each output frame is a
// windowed-sinc FIR evaluated at an arbitrary fractional input position; the
// coefficients come from a table of kPhases filter phases, interpolated
// linearly between neighbouring phases.
//
// Besides resampling, it reports what an adaptive-resampling control loop
// needs (Adriaensen, "Controlling adaptive resampling", section 3.2): where
// the next output frame lies relative to the input read so far, fraction
// included. Integer frame counters miss that fraction, and with equal nominal
// rates the miss becomes a slow sawtooth the loop cannot filter out.
//
// Positions are in input frames. The first output after reset() lands exactly
// on the first input frame read after it; every output advances the position
// by the ratio in effect at the time.
class Resampler {
public:
    static constexpr std::size_t kMaxHalfLength = 64;
    static constexpr std::size_t kPhases = 256;
    static constexpr double kCutoff = 0.9;          // of the lower Nyquist frequency
    static constexpr double kKaiserBeta = 8.0;      // about 80 dB stopband

    // Allocates: call before the audio thread runs, never on it.
    // `nominalRatio` is input frames per output frame; it sets the cutoff, so
    // downsampling does not alias. Ends in reset().
    void configure(std::size_t channels, double nominalRatio, std::size_t halfLength = 32) {
        channels_ = channels;
        half_ = std::clamp<std::size_t>(halfLength, 1, kMaxHalfLength);
        taps_ = 2 * half_;
        ratio_ = nominalRatio;
        buildTable(kCutoff * std::min(1.0, 1.0 / nominalRatio));
        history_.assign(2 * taps_ * channels_, 0.0f);
        reset();
    }

    // Silent history. The first output then needs half the filter length of
    // input ahead of it, which the first process() reads.
    void reset() noexcept {
        std::fill(history_.begin(), history_.end(), 0.0f);
        head_ = 0;
        frac_ = 0.0;
        pending_ = half_ + 1;
    }

    // Input frames per output frame, from the next output on.
    void setRatio(double inputPerOutput) noexcept {
        ratio_ = inputPerOutput;
    }

    [[nodiscard]] double ratio() const noexcept {
        return ratio_;
    }

    [[nodiscard]] std::size_t halfLength() const noexcept {
        return half_;
    }

    // Input frames the next process() for `outputFrames` frames needs, exactly.
    // Only valid until the ratio changes.
    [[nodiscard]] std::size_t inputFor(std::size_t outputFrames) const noexcept {
        if (outputFrames == 0) {
            return 0;
        }
        std::size_t need = pending_;
        double frac = frac_;
        for (std::size_t i = 1; i < outputFrames; ++i) {
            need += advance(frac);
        }
        return need;
    }

    // `inFrames` must be inputFor(outFrames).
    void process(const std::int16_t* in, std::size_t inFrames,
                 std::int16_t* out, std::size_t outFrames) noexcept {
        process(in, inFrames, nullptr, 0, out, outFrames);
    }

    // The same, with the input split in two, as a ring buffer hands it out:
    // `in1` first, then `in2`. `n1 + n2` must be inputFor(outFrames).
    void process(const std::int16_t* in1, std::size_t n1,
                 const std::int16_t* in2, std::size_t n2,
                 std::int16_t* out, std::size_t outFrames) noexcept {
        for (std::size_t o = 0; o < outFrames; ++o) {
            for (; pending_ > 0; --pending_) {
                if (n1 == 0) {
                    in1 = in2;
                    n1 = n2;
                    n2 = 0;
                }
                push(in1);
                in1 += channels_;
                --n1;
            }
            compute(out);
            out += channels_;
            pending_ = advance(frac_);
        }
    }

    // How far, in input frames, the next output lies behind the input read so
    // far. Settles around the half filter length plus one while running: that
    // much input sits in the filter. The paper's d_res.
    [[nodiscard]] double inputDistance() const noexcept {
        return static_cast<double>(half_ + 1) - frac_ - static_cast<double>(pending_);
    }

private:
    // Moves `frac` on by one output; returns the whole input frames passed.
    [[nodiscard]] std::size_t advance(double& frac) const noexcept {
        frac += ratio_;
        const double whole = std::floor(frac);
        frac -= whole;
        return static_cast<std::size_t>(whole);
    }

    // History holds every frame twice, taps_ apart, so the last taps_ frames
    // are always contiguous, starting at head_.
    void push(const std::int16_t* frame) noexcept {
        float* a = history_.data() + head_ * channels_;
        float* b = a + taps_ * channels_;
        for (std::size_t c = 0; c < channels_; ++c) {
            a[c] = b[c] = static_cast<float>(frame[c]) * (1.0f / 32768.0f);
        }
        head_ = head_ + 1 == taps_ ? 0 : head_ + 1;
    }

    // The output sits between window frames half_ - 1 and half_, at frac_.
    void compute(std::int16_t* out) const noexcept {
        const double pos = frac_ * static_cast<double>(kPhases);
        const auto phase = std::min(static_cast<std::size_t>(pos), kPhases - 1);
        const auto mix = static_cast<float>(pos - static_cast<double>(phase));
        const float* row0 = table_.data() + phase * taps_;
        const float* row1 = row0 + taps_;

        std::array<float, 2 * kMaxHalfLength> coef;
        for (std::size_t j = 0; j < taps_; ++j) {
            coef[j] = row0[j] + mix * (row1[j] - row0[j]);
        }

        const float* window = history_.data() + head_ * channels_;
        for (std::size_t c = 0; c < channels_; ++c) {
            float sum = 0.0f;
            for (std::size_t j = 0; j < taps_; ++j) {
                sum += coef[j] * window[j * channels_ + c];
            }
            const long v = std::lrint(sum * 32768.0f);
            out[c] = static_cast<std::int16_t>(std::clamp(v, -32768L, 32767L));
        }
    }

    // Row p holds the filter for fraction p / kPhases; the extra last row
    // (fraction 1) is the interpolation partner of the one before it.
    void buildTable(double cutoff) {
        table_.assign((kPhases + 1) * taps_, 0.0f);
        const double half = static_cast<double>(half_);
        for (std::size_t p = 0; p <= kPhases; ++p) {
            const double frac = static_cast<double>(p) / static_cast<double>(kPhases);
            std::array<double, 2 * kMaxHalfLength> h;
            double sum = 0.0;
            for (std::size_t j = 0; j < taps_; ++j) {
                const double t = static_cast<double>(j) - (half - 1.0) - frac;
                h[j] = cutoff * sinc(cutoff * t) * kaiser(t / half);
                sum += h[j];
            }
            // Unity gain at DC in every phase, or the level would ripple with the fraction.
            float* row = table_.data() + p * taps_;
            for (std::size_t j = 0; j < taps_; ++j) {
                row[j] = static_cast<float>(h[j] / sum);
            }
        }
    }

    static double sinc(double x) noexcept {
        if (std::abs(x) < 1e-12) {
            return 1.0;
        }
        const double px = std::numbers::pi * x;
        return std::sin(px) / px;
    }

    static double kaiser(double x) noexcept {
        if (std::abs(x) >= 1.0) {
            return 0.0;
        }
        return besselI0(kKaiserBeta * std::sqrt(1.0 - x * x)) / besselI0(kKaiserBeta);
    }

    // Power series; std::cyl_bessel_i is missing from libc++.
    static double besselI0(double x) noexcept {
        double sum = 1.0, term = 1.0;
        const double q = x * x / 4.0;
        for (int k = 1; k < 50 && term > 1e-12 * sum; ++k) {
            term *= q / (static_cast<double>(k) * static_cast<double>(k));
            sum += term;
        }
        return sum;
    }

    std::size_t channels_ = 0;
    std::size_t half_ = 0;
    std::size_t taps_ = 0;
    double ratio_ = 1.0;

    std::vector<float> table_;
    std::vector<float> history_;
    std::size_t head_ = 0;
    double frac_ = 0.0;             // where the next output lies between two input frames
    std::size_t pending_ = 0;       // input frames to read before the next output
};

#endif  // RESAMPLER_H
