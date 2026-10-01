#include "presentation_timeline.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <limits>

using Catch::Approx;

namespace {

constexpr std::uint64_t kFrames = 240;
constexpr double kPeriod = 0.005;

// Reports for packets 0..n-1: packet i is due at 1 + i * kPeriod and starts
// at ring position 1000 + i * kFrames.
PresentationTimeline makeTimeline(int n) {
    PresentationTimeline timeline;
    for (int i = 0; i < n; ++i) {
        timeline.add({1.0 + i * kPeriod, 1000 + static_cast<std::uint64_t>(i) * kFrames, false});
    }
    return timeline;
}

}  // namespace

TEST_CASE("nothing before two reports", "[timeline]") {
    PresentationTimeline timeline;
    REQUIRE_FALSE(timeline.framesAhead(1.0, 0));
    timeline.add({1.0, 1000, false});
    REQUIRE_FALSE(timeline.framesAhead(1.0, 0));
    timeline.add({1.005, 1240, false});
    REQUIRE(timeline.framesAhead(1.0, 0));
}

TEST_CASE("the frame due is relative to the position asked about", "[timeline]") {
    PresentationTimeline timeline = makeTimeline(2);
    REQUIRE(*timeline.framesAhead(1.0, 1000) == Approx(0.0));
    REQUIRE(*timeline.framesAhead(1.0, 900) == Approx(100.0));
    REQUIRE(*timeline.framesAhead(1.0, 1100) == Approx(-100.0));
}

TEST_CASE("interpolates between the reports around the time asked about", "[timeline]") {
    // The middle report is off the line through the others: only the pair
    // around the time asked about may be used.
    PresentationTimeline timeline;
    timeline.add({1.000, 1000, false});
    timeline.add({1.004, 1240, false});     // due early
    timeline.add({1.010, 1480, false});

    REQUIRE(*timeline.framesAhead(1.002, 1000) == Approx(120.0));          // first pair
    REQUIRE(*timeline.framesAhead(1.007, 1000) == Approx(240.0 + 120.0));  // second pair
}

TEST_CASE("extrapolates before the oldest report and past the newest", "[timeline]") {
    PresentationTimeline timeline = makeTimeline(3);
    // Not due yet: the first frame lies a whole packet ahead.
    REQUIRE(*timeline.framesAhead(1.0 - kPeriod, 1000) == Approx(-240.0));
    // The stream stopped: the line runs on.
    REQUIRE(*timeline.framesAhead(1.0 + 4 * kPeriod, 1000) == Approx(4 * 240.0));
}

TEST_CASE("forgets reports older than the pair around the time asked about", "[timeline]") {
    PresentationTimeline timeline = makeTimeline(10);
    REQUIRE(timeline.size() == 10);
    (void)timeline.framesAhead(1.0 + 4.5 * kPeriod, 0);
    REQUIRE(timeline.size() == 6);      // packets 4..9 remain
    (void)timeline.framesAhead(1.0 + 100 * kPeriod, 0);
    REQUIRE(timeline.size() == 2);      // never fewer than a pair
}

TEST_CASE("a restart report starts a new timeline", "[timeline]") {
    PresentationTimeline timeline = makeTimeline(5);
    timeline.add({50.0, 9000, true});
    REQUIRE(timeline.size() == 1);
    REQUIRE_FALSE(timeline.framesAhead(50.0, 9000));
    timeline.add({50.005, 9240, false});
    REQUIRE(*timeline.framesAhead(50.0025, 9000) == Approx(120.0));
}

TEST_CASE("a full history drops its oldest report", "[timeline]") {
    PresentationTimeline timeline = makeTimeline(PresentationTimeline::kCapacity + 3);
    REQUIRE(timeline.size() == PresentationTimeline::kCapacity);
    // Packet 3 is now the oldest; before it, the line extrapolates from 3 and 4.
    REQUIRE(*timeline.framesAhead(1.0, 1000) == Approx(0.0).margin(1e-6));
}

TEST_CASE("positions may wrap", "[timeline]") {
    constexpr std::uint64_t kNearEnd = std::numeric_limits<std::uint64_t>::max() - 100;
    PresentationTimeline timeline;
    timeline.add({1.0, kNearEnd, false});
    timeline.add({1.005, kNearEnd + kFrames, false});     // wraps
    REQUIRE(*timeline.framesAhead(1.0025, kNearEnd - 500) == Approx(500.0 + 120.0));
}

TEST_CASE("nothing from stamps that do not increase", "[timeline]") {
    PresentationTimeline timeline;
    timeline.add({1.0, 1000, false});
    timeline.add({1.0, 1240, false});
    REQUIRE_FALSE(timeline.framesAhead(1.0, 1000));
}
