#include "frame_ring.h"
#include "resampler.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <vector>

using Catch::Approx;

namespace {

constexpr double kRate = 48000.0;
constexpr double kAmplitude = 0.5;

std::int16_t sineAt(double position, double frequency) {
    const double v = kAmplitude * std::sin(2.0 * std::numbers::pi * frequency * position / kRate);
    return static_cast<std::int16_t>(std::lrint(v * 32768.0));
}

// Feeds a mono signal, given as a function of the input frame index, through
// the resampler in blocks of `block` output frames.
template <typename Signal>
std::vector<std::int16_t> run(Resampler& r, std::size_t outputs, std::size_t block, Signal signal,
                              std::uint64_t& read) {
    std::vector<std::int16_t> out(outputs);
    std::vector<std::int16_t> in;
    for (std::size_t done = 0; done < outputs; done += block) {
        const std::size_t n = std::min(block, outputs - done);
        in.resize(r.inputFor(n));
        for (auto& frame : in) {
            frame = signal(read++);
        }
        r.process(in.data(), in.size(), out.data() + done, n);
    }
    return out;
}

}  // namespace

TEST_CASE("no output needs no input", "[resampler]") {
    Resampler r;
    r.configure(1, 1.0);
    REQUIRE(r.inputFor(0) == 0);
}

TEST_CASE("the first output needs half the filter ahead of it", "[resampler]") {
    Resampler r;
    r.configure(2, 1.0, 16);
    REQUIRE(r.inputFor(1) == 17);
    REQUIRE(r.inputDistance() == Approx(0.0));
}

TEST_CASE("input distance tracks the output position exactly", "[resampler]") {
    for (const double ratio : {1.0, 1.0003, 0.9997, 48000.0 / 44100.0, 44100.0 / 48000.0}) {
        Resampler r;
        r.configure(1, ratio);
        std::uint64_t read = 0;
        const std::size_t outputs = 240 * 50;
        (void)run(r, outputs, 240, [](std::uint64_t) { return std::int16_t{0}; }, read);

        // Output n sits at input position n * ratio, the next one at outputs * ratio.
        const double next = static_cast<double>(outputs) * ratio;
        REQUIRE(r.inputDistance() == Approx(static_cast<double>(read) - next).margin(1e-6));
    }
}

TEST_CASE("input distance follows ratio changes", "[resampler]") {
    Resampler r;
    r.configure(1, 1.0);
    std::uint64_t read = 0;
    double position = 0.0;
    for (int block = 0; block < 100; ++block) {
        const double ratio = 1.0 + 0.004 * std::sin(block * 0.1);
        r.setRatio(ratio);
        (void)run(r, 256, 256, [](std::uint64_t) { return std::int16_t{0}; }, read);
        position += 256 * ratio;
    }
    REQUIRE(r.inputDistance() == Approx(static_cast<double>(read) - position).margin(1e-6));
}

TEST_CASE("input distance settles near half the filter length", "[resampler]") {
    Resampler r;
    r.configure(1, 1.0001, 32);
    std::uint64_t read = 0;
    (void)run(r, 4800, 240, [](std::uint64_t) { return std::int16_t{0}; }, read);
    REQUIRE(r.inputDistance() > 29.0);
    REQUIRE(r.inputDistance() <= 33.0);
}

TEST_CASE("a constant passes at unity gain", "[resampler]") {
    Resampler r;
    r.configure(1, 1.00037);
    std::uint64_t read = 0;
    const auto out = run(r, 4800, 240, [](std::uint64_t) { return std::int16_t{10000}; }, read);
    for (std::size_t i = 100; i < out.size(); ++i) {
        REQUIRE(std::abs(out[i] - 10000) <= 2);
    }
}

TEST_CASE("output n is the input signal at position n * ratio", "[resampler]") {
    constexpr double kFrequency = 1000.0;
    for (const double ratio : {1.0, 1.0001, 0.9999, 4.0 / 3.0}) {
        Resampler r;
        r.configure(1, ratio);
        std::uint64_t read = 0;
        const auto out = run(r, 4800, 240, [](std::uint64_t i) { return sineAt(static_cast<double>(i), kFrequency); },
                             read);
        double worst = 0.0;
        for (std::size_t n = 100; n < out.size(); ++n) {
            const double expected = sineAt(static_cast<double>(n) * ratio, kFrequency);
            worst = std::max(worst, std::abs(out[n] - expected));
        }
        INFO("ratio " << ratio << ", worst error " << worst << " LSB");
        REQUIRE(worst < 16.0);      // -66 dB relative to the signal
    }
}

TEST_CASE("channels are resampled independently", "[resampler]") {
    Resampler r;
    r.configure(2, 1.0002);
    std::vector<std::int16_t> in(r.inputFor(480) * 2);
    for (std::size_t i = 0; i < in.size() / 2; ++i) {
        in[2 * i] = 8000;
        in[2 * i + 1] = -8000;
    }
    std::vector<std::int16_t> out(480 * 2);
    r.process(in.data(), in.size() / 2, out.data(), 480);
    for (std::size_t i = 100; i < 480; ++i) {
        REQUIRE(std::abs(out[2 * i] - 8000) <= 2);
        REQUIRE(std::abs(out[2 * i + 1] + 8000) <= 2);
    }
}

TEST_CASE("input split in two gives the same output as in one piece", "[resampler]") {
    constexpr std::size_t kChannels = 2;
    constexpr std::size_t kOutputs = 480;
    constexpr double kRatio = 1.0003;

    Resampler reference;
    reference.configure(kChannels, kRatio);
    const std::size_t need = reference.inputFor(kOutputs);
    std::vector<std::int16_t> in(need * kChannels);
    for (std::size_t i = 0; i < need; ++i) {
        in[kChannels * i] = sineAt(static_cast<double>(i), 1000.0);
        in[kChannels * i + 1] = sineAt(static_cast<double>(i), 3000.0);
    }
    std::vector<std::int16_t> expected(kOutputs * kChannels);
    reference.process(in.data(), need, expected.data(), kOutputs);

    // Every split point, both ends included: an empty first or second part.
    for (std::size_t split = 0; split <= need; ++split) {
        Resampler r;
        r.configure(kChannels, kRatio);
        std::vector<std::int16_t> out(kOutputs * kChannels);
        r.process(in.data(), split, in.data() + split * kChannels, need - split, out.data(), kOutputs);
        INFO("split at " << split << " of " << need);
        REQUIRE(out == expected);
        REQUIRE(r.inputDistance() == reference.inputDistance());
    }
}

TEST_CASE("input read in place across the ring's wrap", "[resampler]") {
    constexpr std::size_t kChannels = 2;
    constexpr std::size_t kOutputs = 256;
    constexpr double kRatio = 0.9997;
    using Ring = FrameRing<1024>;

    Resampler reference;
    reference.configure(kChannels, kRatio);
    const std::size_t need = reference.inputFor(kOutputs);
    std::vector<std::int16_t> in(need * kChannels);
    for (std::size_t i = 0; i < in.size(); ++i) {
        in[i] = sineAt(static_cast<double>(i), 1000.0);
    }
    std::vector<std::int16_t> expected(kOutputs * kChannels);
    reference.process(in.data(), need, expected.data(), kOutputs);

    // Move both counters near the end, so the input wraps about halfway.
    auto ring = std::make_unique<Ring>();
    REQUIRE(ring->init(kChannels * sizeof(std::int16_t)));
    const std::size_t lead = Ring::capacity() - need / 2;
    REQUIRE(ring->writeSilence(lead));
    REQUIRE(ring->discard(lead));
    REQUIRE(ring->write(in.data(), need));

    const Regions regions = ring->acquireRead(need);
    REQUIRE(regions.frames() == need);
    REQUIRE(regions.region1().len == need / 2);
    REQUIRE(regions.region2().len == need - need / 2);

    Resampler r;
    r.configure(kChannels, kRatio);
    std::vector<std::int16_t> out(kOutputs * kChannels);
    r.process(reinterpret_cast<const std::int16_t*>(regions.region1().buf), regions.region1().len,
              reinterpret_cast<const std::int16_t*>(regions.region2().buf), regions.region2().len,
              out.data(), kOutputs);
    REQUIRE(ring->commitRead(need));
    REQUIRE(out == expected);
}
