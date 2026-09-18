#include "servo.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>

using Catch::Approx;
using namespace std::chrono_literals;

namespace {

constexpr double kInterval = 0.125;   // 8 samples/s, the production rate
constexpr double kOffset   = 1234.5;  // master - local at local == 0
constexpr double kSkew     = 20e-6;   // 20 ppm

constexpr double masterAt(double local) noexcept {
    return kOffset + local * (1.0 + kSkew);
}

/// A configured servo whose mapping has just been engaged by the 8 seed
/// samples, all with the same delay. Returns the local time of the next slot.
double seed(Servo& servo, std::chrono::nanoseconds delay = 1ms) {
    servo.configure(0.5, 0.05);
    double local = 0.0;
    for (int i = 0; i < 8; ++i) {
        servo.addSample(local, masterAt(local), delay);
        local += kInterval;
    }
    return local;
}

}  // namespace

TEST_CASE("localToMaster and masterToLocal are inverses", "[servo]") {
    const ClockMapping mapping{.localRef = 10.0, .masterRef = 1000.0, .skew = 37e-6};

    REQUIRE(localToMaster(mapping.localRef, mapping) == Approx(mapping.masterRef));
    REQUIRE(masterToLocal(mapping.masterRef, mapping) == Approx(mapping.localRef));

    for (double local : {0.0, 10.0, 12.5, 1e4}) {
        const double master = localToMaster(local, mapping);
        REQUIRE(masterToLocal(master, mapping) == Approx(local).margin(1e-9));
    }
}

TEST_CASE("a fresh servo is unsynced", "[servo]") {
    Servo servo;
    servo.configure(0.5, 0.05);

    REQUIRE(servo.state().value == State::Unsynced);
    REQUIRE_FALSE(servo.mapping().has_value());
    REQUIRE(servo.rejected() == 0);
    REQUIRE(servo.tooSoon() == 0);
}

TEST_CASE("the servo seeds from eight samples and picks the lowest delay", "[servo]") {
    Servo servo;
    servo.configure(0.5, 0.05);

    // Delays fall steadily, so the last seed is the one that should win.
    double local = 0.0;
    for (int i = 0; i < 7; ++i) {
        servo.addSample(local, masterAt(local), std::chrono::milliseconds{10 - i});
        REQUIRE_FALSE(servo.mapping().has_value());
        REQUIRE(servo.state().value == State::Unsynced);
        local += kInterval;
    }

    servo.addSample(local, masterAt(local), 3ms);

    REQUIRE(servo.mapping().has_value());
    REQUIRE(servo.state().value == State::Acquiring);
    REQUIRE(servo.mapping()->localRef == Approx(local));
    REQUIRE(servo.mapping()->masterRef == Approx(masterAt(local)));
    REQUIRE(servo.mapping()->skew == Approx(0.0));
}

TEST_CASE("the servo locks and converges on the injected skew", "[servo]") {
    Servo servo;
    double local = seed(servo);

    REQUIRE(servo.state().value == State::Acquiring);

    // Lock needs both >8 s since acquisition started and >=32 accepted samples.
    constexpr int kSamples = 800;  // 100 s at 8/s
    for (int i = 0; i < kSamples; ++i) {
        servo.addSample(local, masterAt(local), 1ms);
        local += kInterval;
    }

    REQUIRE(servo.state().value == State::Locked);
    REQUIRE(servo.rejected() == 0);
    REQUIRE(servo.tooSoon() == 0);

    const auto mapping = servo.mapping();
    REQUIRE(mapping.has_value());
    REQUIRE(mapping->skew == Approx(kSkew).margin(2e-6));

    // The property that actually matters: the mapping predicts master time.
    for (double ahead : {0.0, 1.0, 10.0}) {
        const double t = local + ahead;
        REQUIRE(localToMaster(t, *mapping) == Approx(masterAt(t)).margin(1e-5));
    }
}

TEST_CASE("the gate is computed before it is in force", "[servo]") {
    Servo servo;
    servo.configure(0.5, 0.05);

    // kMinForGate is 16. One sample is enough to produce a threshold, so the
    // sentinel is gone long before anything is actually gated by it.
    double local = 0.0;
    servo.addSample(local, masterAt(local), 1ms);
    local += kInterval;
    REQUIRE(servo.gateThreshold() != std::chrono::nanoseconds::max());
    REQUIRE_FALSE(servo.gateActive());

    for (int i = 1; i < 15; ++i) {
        servo.addSample(local, masterAt(local), 1ms);
        local += kInterval;
    }
    REQUIRE_FALSE(servo.gateActive());

    servo.addSample(local, masterAt(local), 1ms);
    REQUIRE(servo.gateActive());
}

TEST_CASE("the delay gate rejects outliers once the window is large enough", "[servo]") {
    Servo servo;
    servo.configure(0.5, 0.05);

    // Alternating delays give the window a known shape: min 1 ms, median 3 ms,
    // so the gate should settle at min + 2*(median - min) == 5 ms.
    double local = 0.0;
    for (int i = 0; i < 28; ++i) {
        servo.addSample(local, masterAt(local), (i % 2 == 0) ? 1ms : 3ms);
        local += kInterval;
    }

    REQUIRE(servo.rejected() == 0);
    REQUIRE(servo.pathDelay() == std::chrono::nanoseconds{1ms});
    REQUIRE(servo.gateThreshold() == std::chrono::nanoseconds{5ms});

    SECTION("a sample inside the threshold is accepted") {
        const auto before = servo.mapping();
        REQUIRE(before.has_value());

        servo.addSample(local, masterAt(local), 4ms);

        REQUIRE(servo.rejected() == 0);
        REQUIRE(servo.mapping()->localRef == Approx(local));
        REQUIRE(servo.mapping()->localRef != Approx(before->localRef));
    }

    SECTION("a sample beyond the threshold is rejected and changes nothing") {
        const auto before = servo.mapping();
        REQUIRE(before.has_value());

        // Grossly delayed, and carrying a master time that would wreck the
        // mapping if it were let through.
        servo.addSample(local, masterAt(local) + 0.5, 6ms);

        REQUIRE(servo.rejected() == 1);
        const auto after = servo.mapping();
        REQUIRE(after.has_value());
        REQUIRE(after->localRef == Approx(before->localRef));
        REQUIRE(after->masterRef == Approx(before->masterRef));
        REQUIRE(after->skew == Approx(before->skew));
    }
}

TEST_CASE("samples arriving too soon are counted and ignored", "[servo]") {
    Servo servo;
    const double local = seed(servo);

    const auto before = servo.mapping();
    REQUIRE(before.has_value());

    // Less than 10 ms since the reference: the interval is too short for a
    // meaningful frequency estimate.
    servo.addSample(before->localRef + 0.005, masterAt(local), 1ms);

    REQUIRE(servo.tooSoon() == 1);
    REQUIRE(servo.mapping()->localRef == Approx(before->localRef));
}

TEST_CASE("five consecutive large errors reset the servo", "[servo]") {
    Servo servo;
    double local = seed(servo);

    // A full second off — far beyond the 1 ms tolerance, and far more than the
    // loop can pull in per sample, so every one of these counts as large.
    for (int i = 0; i < 4; ++i) {
        servo.addSample(local, masterAt(local) + 1.0, 1ms);
        REQUIRE(servo.mapping().has_value());
        local += kInterval;
    }

    servo.addSample(local, masterAt(local) + 1.0, 1ms);

    REQUIRE_FALSE(servo.mapping().has_value());
    REQUIRE(servo.state().value == State::Unsynced);
}

TEST_CASE("reset clears the mapping, the delay window and the counters", "[servo]") {
    Servo servo;
    double local = seed(servo);

    for (int i = 0; i < 20; ++i) {
        servo.addSample(local, masterAt(local), 1ms);
        local += kInterval;
    }
    servo.addSample(local, masterAt(local), 50ms);   // rejected
    local += kInterval;
    servo.addSample(local, masterAt(local), 1ms);
    servo.addSample(local + 0.001, masterAt(local), 1ms);  // too soon

    REQUIRE(servo.rejected() == 1);
    REQUIRE(servo.tooSoon() == 1);

    servo.reset();

    REQUIRE_FALSE(servo.mapping().has_value());
    REQUIRE(servo.state().value == State::Unsynced);
    REQUIRE(servo.rejected() == 0);
    REQUIRE(servo.tooSoon() == 0);
    REQUIRE(servo.pathDelay() == std::chrono::nanoseconds{0});
    REQUIRE(servo.gateThreshold() == std::chrono::nanoseconds::max());
}
