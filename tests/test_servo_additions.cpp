// Coverage for the per-leg delay gate.
//
// Every case in test_servo.cpp passes a single `delay`, so it goes through the
// symmetric overload and a == b. With equal legs the two windows are identical
// by construction, so none of those cases can reach the per-leg logic. These
// two do.

#include "servo.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>

using namespace std::chrono_literals;

namespace {

constexpr double kInterval = 0.125;   // 8 samples/s, the production rate
constexpr double kOffset   = 1234.5;  // master - local at local == 0
constexpr double kSkew     = 20e-6;   // 20 ppm

constexpr double masterAt(double local) noexcept {
    return kOffset + local * (1.0 + kSkew);
}

}  // namespace

TEST_CASE("per-leg gate rejects an asymmetric sample the mean would pass",
          "[servo][gate]") {
    Servo servo;
    servo.configure(0.5, 0.05);

    double local = 0.0;

    // A steady symmetric baseline, long enough to arm the gate and settle both
    // floors at 1 ms.
    for (int i = 0; i < 40; ++i) {
        servo.addSample(local, masterAt(local), 1ms, 1ms);
        local += kInterval;
    }
    REQUIRE(servo.gateActive());
    const auto rejectedBefore = servo.rejected();

    // One leg at its floor, the other far above it. The mean is (1 + 9)/2 = 5 ms,
    // which a gate on the average lets through; the per-leg gate sees leg B
    // well outside its own threshold.
    servo.addSample(local, masterAt(local), 1ms, 9ms);
    local += kInterval;
    CHECK(servo.rejected() == rejectedBefore + 1);

    // The mirror image, so the test is not accidentally direction-specific.
    servo.addSample(local, masterAt(local), 9ms, 1ms);
    local += kInterval;
    CHECK(servo.rejected() == rejectedBefore + 2);

    // A symmetric sample at the floor still passes.
    servo.addSample(local, masterAt(local), 1ms, 1ms);
    CHECK(servo.rejected() == rejectedBefore + 2);
}

TEST_CASE("per-leg floors expose path asymmetry", "[servo][gate]") {
    Servo servo;
    servo.configure(0.5, 0.05);

    double local = 0.0;

    // A constant 400 us asymmetry on every exchange. This is the shape the gate
    // provably cannot remove -- it lives in the floors, not in the queueing
    // tail -- so floorA/floorB is the only way it becomes visible.
    for (int i = 0; i < 40; ++i) {
        servo.addSample(local, masterAt(local), 1000us, 1400us);
        local += kInterval;
    }

    CHECK(servo.floorA() == 1000us);
    CHECK(servo.floorB() == 1400us);

    // The systematic offset a two-way exchange is structurally unable to
    // detect is half the difference between the legs.
    CHECK((servo.floorB() - servo.floorA()) / 2 == 200us);

    // pathDelay() still reports the combined figure, for sizing a buffer.
    CHECK(servo.pathDelay() == 1200us);
}