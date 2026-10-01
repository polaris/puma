#include "time_filter.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using Catch::Approx;

namespace {

constexpr double   kNominalRate     = 48000.0;
constexpr ma_uint32 kFramesPerPeriod = 480;          // 10 ms
constexpr double   kNominalPeriod   = static_cast<double>(kFramesPerPeriod) / kNominalRate;
constexpr double   kBandwidth       = 0.5;           // Hz

TimeFilter makeFilter(double startTime = 100.0) {
    TimeFilter filter;
    filter.configure(kBandwidth, kFramesPerPeriod, kNominalRate);
    filter.reset(startTime);
    return filter;
}

}  // namespace

TEST_CASE("a filter is not ready until it is reset", "[time_filter]") {
    TimeFilter filter;
    REQUIRE_FALSE(filter.ready());

    filter.configure(kBandwidth, kFramesPerPeriod, kNominalRate);
    REQUIRE_FALSE(filter.ready());

    filter.reset(0.0);
    REQUIRE(filter.ready());

    filter.invalidate();
    REQUIRE_FALSE(filter.ready());
}

TEST_CASE("reset starts from the nominal period", "[time_filter]") {
    constexpr double kStart = 100.0;
    TimeFilter filter = makeFilter(kStart);

    REQUIRE(filter.time() == Approx(kStart));
    REQUIRE(filter.frame() == 0);
    REQUIRE(filter.period() == Approx(kNominalPeriod));
    REQUIRE(filter.rate() == Approx(kNominalRate));
    REQUIRE(filter.error() == Approx(0.0));
    REQUIRE(filter.framesPerPeriod() == kFramesPerPeriod);
}

TEST_CASE("a perfectly nominal clock leaves the estimate alone", "[time_filter]") {
    constexpr double kStart = 100.0;
    TimeFilter filter = makeFilter(kStart);

    for (int i = 1; i <= 100; ++i) {
        filter.update(kStart + static_cast<double>(i) * kNominalPeriod);
        REQUIRE(filter.error() == Approx(0.0).margin(1e-12));
    }

    REQUIRE(filter.period() == Approx(kNominalPeriod).margin(1e-12));
    REQUIRE(filter.rate() == Approx(kNominalRate).margin(1e-6));
}

TEST_CASE("update advances the frame counter by one period", "[time_filter]") {
    constexpr double kStart = 0.0;
    TimeFilter filter = makeFilter(kStart);

    for (std::uint64_t i = 1; i <= 10; ++i) {
        filter.update(kStart + static_cast<double>(i) * kNominalPeriod);
        REQUIRE(filter.frame() == i * kFramesPerPeriod);
    }
}

TEST_CASE("skip accounts for missed periods", "[time_filter]") {
    constexpr double kStart = 0.0;
    constexpr std::uint32_t kMissing = 3;

    TimeFilter filter = makeFilter(kStart);
    filter.update(kStart + kNominalPeriod);

    const std::uint64_t frameBefore = filter.frame();
    const double periodBefore = filter.period();

    filter.skip(kMissing);
    REQUIRE(filter.period() == Approx(periodBefore));

    // The gap shows up on the next real callback: the frame counter has moved
    // on by the missed periods plus this one, and the filter has not treated
    // the long gap as a timing error.
    filter.update(kStart + static_cast<double>(2 + kMissing) * kNominalPeriod);

    REQUIRE(filter.frame() == frameBefore + (kMissing + 1) * kFramesPerPeriod);
    REQUIRE(filter.error() == Approx(0.0).margin(1e-12));
    REQUIRE(filter.period() == Approx(periodBefore).margin(1e-12));
}

TEST_CASE("the filter converges on a clock running off nominal", "[time_filter]") {
    constexpr double kTrueRate = 48000.5;               // 10 ppm fast
    constexpr double kTruePeriod = static_cast<double>(kFramesPerPeriod) / kTrueRate;
    constexpr double kStart = 100.0;

    TimeFilter filter = makeFilter(kStart);

    // The first callback already sees the mismatch.
    filter.update(kStart + kTruePeriod);
    const double firstError = std::abs(filter.error());
    REQUIRE(firstError > 0.0);

    for (int i = 2; i <= 2000; ++i) {  // 20 s at 10 ms
        filter.update(kStart + static_cast<double>(i) * kTruePeriod);
    }

    REQUIRE(filter.rate() == Approx(kTrueRate).margin(0.05));
    REQUIRE(filter.period() == Approx(kTruePeriod).margin(1e-9));
    REQUIRE(std::abs(filter.error()) < firstError);
    REQUIRE(std::abs(filter.error()) < 1e-6);
}

TEST_CASE("a fast start settles sooner, then runs at the bandwidth", "[time_filter]") {
    // The true period is 0.1% longer than nominal.
    const double truePeriod = kNominalPeriod * 1.001;

    TimeFilter fast, plain;
    fast.configure(kBandwidth, kFramesPerPeriod, kNominalRate, 5.0, 1.0);
    plain.configure(kBandwidth, kFramesPerPeriod, kNominalRate);
    fast.reset(0.0);
    plain.reset(0.0);
    for (int i = 1; i <= 100; ++i) {        // one second
        fast.update(i * truePeriod);
        plain.update(i * truePeriod);
    }
    REQUIRE(std::abs(fast.period() - truePeriod) < 0.1 * std::abs(plain.period() - truePeriod));

    // Past the start, the same disturbance moves both by the same amount.
    for (int i = 101; i <= 2000; ++i) {
        fast.update(i * truePeriod);
        plain.update(i * truePeriod);
    }
    const double fastBefore = fast.period();
    const double plainBefore = plain.period();
    fast.update(2001 * truePeriod + 1e-3);
    plain.update(2001 * truePeriod + 1e-3);
    REQUIRE(fast.period() - fastBefore == Approx(plain.period() - plainBefore).epsilon(1e-6));
}
