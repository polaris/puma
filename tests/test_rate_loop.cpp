#include "presentation_timeline.h"
#include "rate_loop.h"
#include "resampler.h"
#include "servo.h"
#include "time_filter.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

using Catch::Approx;

namespace {

constexpr std::size_t kFrames = 240;    // per callback and per packet
constexpr std::size_t kNeed = 241;      // what a read takes
constexpr std::uint64_t kPlenty = 10000;

RateLoop::Config makeConfig() {
    RateLoop::Config config;
    config.nominalRatio = 1.0;
    config.deviceRate = 48000.0;
    config.framesPerCallback = kFrames;
    return config;
}

RateLoop makeLoop(RateLoop::Config config = makeConfig()) {
    RateLoop loop;
    loop.configure(config);
    return loop;
}

// A loop that has just started, with nothing trimmed.
RateLoop makeStarted(RateLoop::Config config = makeConfig()) {
    RateLoop loop = makeLoop(config);
    REQUIRE(loop.update(1.0, 0.0, kPlenty, kNeed).play);
    return loop;
}

}  // namespace

TEST_CASE("no step without an error", "[rate_loop]") {
    RateLoop loop = makeLoop();
    const auto step = loop.update(1.0, std::nullopt, kPlenty, kNeed);
    REQUIRE_FALSE(step.play);
    REQUIRE(step.trim == 0);
    REQUIRE(loop.phase() == RateLoop::Phase::Waiting);
}

TEST_CASE("waits while the due frame lies ahead", "[rate_loop]") {
    RateLoop loop = makeLoop();
    const auto step = loop.update(1.0, -0.5, kPlenty, kNeed);
    REQUIRE_FALSE(step.play);
    REQUIRE(step.trim == 0);
    REQUIRE(loop.phase() == RateLoop::Phase::Waiting);
}

TEST_CASE("starts by trimming the frames already past due", "[rate_loop]") {
    RateLoop loop = makeLoop();
    const auto step = loop.update(1.0, 10.4, kPlenty, kNeed);
    REQUIRE(step.play);
    REQUIRE(step.trim == 10);
    REQUIRE_FALSE(step.late);
    REQUIRE(loop.error() == Approx(0.4));
    REQUIRE(loop.phase() == RateLoop::Phase::Settling);
}

TEST_CASE("starts only with a whole read in the ring after the trim", "[rate_loop]") {
    // The slack covers what the correction can add to the read: 241 * 0.5%, rounded up, plus one.
    constexpr std::uint64_t kSlack = 3;
    RateLoop loop = makeLoop();
    REQUIRE_FALSE(loop.update(1.0, 10.0, 10 + kNeed + kSlack - 1, kNeed).play);
    REQUIRE(loop.update(1.0, 10.0, 10 + kNeed + kSlack, kNeed).play);
}

TEST_CASE("discards frames that arrived after they were due", "[rate_loop]") {
    RateLoop loop = makeLoop();
    const auto step = loop.update(1.0, 500.0, 300, kNeed);
    REQUIRE_FALSE(step.play);
    REQUIRE(step.trim == 300);
    REQUIRE(step.late);
    REQUIRE(loop.phase() == RateLoop::Phase::Waiting);
}

TEST_CASE("late reads faster, early slower", "[rate_loop]") {
    SECTION("late") {
        RateLoop loop = makeStarted();
        REQUIRE(loop.update(1.005, 50.0, kPlenty, kNeed).ratio > 1.0);
    }
    SECTION("early") {
        RateLoop loop = makeStarted();
        const auto step = loop.update(1.005, -50.0, kPlenty, kNeed);
        REQUIRE(step.play);     // once started, being early is the loop's to fix
        REQUIRE(step.ratio < 1.0);
    }
}

TEST_CASE("the ratio is relative to the nominal ratio", "[rate_loop]") {
    auto config = makeConfig();
    config.nominalRatio = 44100.0 / 48000.0;
    RateLoop loop = makeLoop(config);
    const auto step = loop.update(1.0, 0.0, kPlenty, kNeed);
    REQUIRE(step.play);
    REQUIRE(step.ratio == Approx(44100.0 / 48000.0));
}

TEST_CASE("the correction is limited", "[rate_loop]") {
    RateLoop loop = makeStarted();
    RateLoop::Step step;
    for (int i = 0; i < 1000; ++i) {
        step = loop.update(1.0, 200.0, kPlenty, kNeed);
    }
    REQUIRE(step.ratio == Approx(1.005));
}

TEST_CASE("settling ends after the start time", "[rate_loop]") {
    RateLoop loop = makeStarted();
    REQUIRE(loop.phase() == RateLoop::Phase::Settling);
    (void)loop.update(1.0 + 3.9, 0.0, kPlenty, kNeed);
    REQUIRE(loop.phase() == RateLoop::Phase::Settling);
    (void)loop.update(1.0 + 4.0, 0.0, kPlenty, kNeed);
    REQUIRE(loop.phase() == RateLoop::Phase::Running);
}

TEST_CASE("realigns instead of steering a large error away", "[rate_loop]") {
    SECTION("late: trims to the due frame") {
        RateLoop loop = makeStarted();
        const auto step = loop.update(1.005, 300.0, kPlenty, kNeed);
        REQUIRE(loop.realigns() == 1);
        REQUIRE(step.play);
        REQUIRE(step.trim == 300);
    }
    SECTION("early: waits for the due frame") {
        RateLoop loop = makeStarted();
        const auto step = loop.update(1.005, -300.0, kPlenty, kNeed);
        REQUIRE(loop.realigns() == 1);
        REQUIRE_FALSE(step.play);
        REQUIRE(loop.phase() == RateLoop::Phase::Waiting);
    }
    SECTION("within the threshold: steers") {
        RateLoop loop = makeStarted();
        REQUIRE(loop.update(1.005, 239.0, kPlenty, kNeed).trim == 0);
        REQUIRE(loop.realigns() == 0);
    }
}

TEST_CASE("losing the error starts over", "[rate_loop]") {
    RateLoop loop = makeStarted();
    REQUIRE_FALSE(loop.update(1.005, std::nullopt, kPlenty, kNeed).play);
    REQUIRE(loop.phase() == RateLoop::Phase::Waiting);
}

TEST_CASE("a restart keeps the learned correction", "[rate_loop]") {
    RateLoop loop = makeStarted();
    for (int i = 0; i < 200; ++i) {
        (void)loop.update(1.0, 20.0, kPlenty, kNeed);
    }
    const double learned = loop.correction();
    REQUIRE(learned > 0.0);

    loop.restart();
    const auto step = loop.update(1.0, 0.0, kPlenty, kNeed);
    REQUIRE(step.play);
    REQUIRE(step.ratio > 1.0);
    REQUIRE(loop.correction() < learned);   // the proportional part is gone, the integral stays
}

// The whole chain in miniature, timed in master time:
// - a sender whose audio clock is off by `ppm`, stamping packets with a
//   smoothed capture time plus L, as sender.cpp does;
// - packets arriving with network delay and jitter;
// - a receiver whose steady clock is off from master time by an offset and
//   `skew`, with an exact clock mapping;
// - a device that is heard at exact intervals of its own clock but calls back
//   with wakeup jitter, smoothed by a TimeFilter;
// - the timeline, the loop and a real resampler, as onPlayback() runs them.
// Only frame counts move; the audio is silence.
//
// The check that matters does not trust the loop's own error: it compares the
// frame actually heard with the frame the sender meant to be heard then.
namespace {

struct SimResult {
    int underruns = 0;              // after the first start
    double meanCorrection = 0.0;    // over the last 60 s
    double maxAbsError = 0.0;       // the loop's own, over the last 60 s, frames
    double maxAbsTruth = 0.0;       // frame heard - frame due, over the last 60 s, frames
    double meanTruth = 0.0;
    double sdTruth = 0.0;
    double startedAt = 0.0;         // master time of the first start
};

struct SimCase {
    double senderRate = 48000;
    double deviceRate = 48000;
    double ppm = 0;                 // sender audio clock against master time
    double skew = 0;                // receiver steady clock against master time
    double offset = 0;              // receiver steady clock minus master time at master 0, seconds
    double networkDelay = 0.001;    // seconds, before jitter
};

SimResult simulate(const SimCase& c, double seconds) {
    constexpr double kBandwidth = 0.05;
    constexpr double kLatency = 0.1;            // L
    constexpr double kCaptureJitter = 0.002;    // seconds, sender callbacks late by up to this
    constexpr double kNetworkJitter = 0.002;
    constexpr double kWakeupJitter = 0.0003;
    constexpr double kStart = 1.0;

    std::mt19937 rng{42};
    std::uniform_real_distribution<double> capture{0.0, kCaptureJitter};
    std::uniform_real_distribution<double> network{0.0, kNetworkJitter};
    std::uniform_real_distribution<double> wakeup{0.0, kWakeupJitter};

    const double nominal = c.senderRate / c.deviceRate;
    const double trueSenderRate = c.senderRate * (1.0 + c.ppm * 1e-6);    // frames per master second

    // local = offset + master * (1 + skew), as the receiver's mapping knows it.
    const auto toLocal = [&](double master) { return c.offset + master * (1.0 + c.skew); };
    const auto toMaster = [&](double local) { return (local - c.offset) / (1.0 + c.skew); };
    const ClockMapping mapping{.localRef = c.offset, .masterRef = 0.0, .skew = 1.0 / (1.0 + c.skew) - 1.0};

    Resampler resampler;
    resampler.configure(1, nominal, 8);

    RateLoop loop;
    RateLoop::Config config;
    config.nominalRatio = nominal;
    config.deviceRate = c.deviceRate;
    config.framesPerCallback = kFrames;
    config.realignFrames = 0.005 * c.senderRate;
    loop.configure(config);

    PresentationTimeline timeline;
    TimeFilter senderClock, deviceClock;
    senderClock.configure(kBandwidth, kFrames, c.senderRate);
    deviceClock.configure(kBandwidth, kFrames, c.deviceRate);

    std::uint64_t written = 0, read = 0;
    std::vector<std::int16_t> in(4096), out(kFrames);
    SimResult result;
    bool started = false;
    double sumCorrection = 0.0, sumTruth = 0.0, sumTruth2 = 0.0;
    int counted = 0;

    // Each packet's and each callback's times are drawn once, when it comes
    // up; drawing them again on every pass would favour early draws.
    std::uint64_t packet = 0, callback = 0;
    double captured = 0.0, arrival = 0.0;
    const auto nextPacket = [&] {
        // The sender's callback for this packet, then its arrival. Packets
        // never overtake each other.
        captured = kStart + static_cast<double>(packet * kFrames) / trueSenderRate + capture(rng);
        arrival = std::max(arrival, captured + c.networkDelay + network(rng));
        ++packet;
    };
    double heardLocal = 0.0, wakeLocal = 0.0, wake = 0.0;
    const auto nextCallback = [&] {
        // The device is heard at exact intervals of its own clock, which runs
        // on the receiver's steady clock; the callback wakes a little late.
        heardLocal = toLocal(kStart) + static_cast<double>(callback * kFrames) / c.deviceRate;
        wakeLocal = heardLocal + wakeup(rng);
        wake = toMaster(wakeLocal);
        ++callback;
    };
    nextPacket();
    nextCallback();

    for (;;) {
        if (std::min(arrival, wake) > seconds) {
            break;
        }

        if (arrival < wake) {
            if (senderClock.ready()) {
                senderClock.update(captured);
            } else {
                senderClock.reset(captured);
            }
            timeline.add({senderClock.time() + kLatency, written, false});
            written += kFrames;
            nextPacket();
            continue;
        }

        const double heardAt = toMaster(heardLocal);
        const double wokeAt = wake;
        if (deviceClock.ready()) {
            deviceClock.update(wakeLocal);
        } else {
            deviceClock.reset(wakeLocal);
        }
        nextCallback();
        const double tD = deviceClock.time();
        // The smoothed wakeups run late of the heard times by their mean
        // lateness: a constant that the output latency calibration takes
        // out in the receiver, here exactly.
        const double heard = localToMaster(tD - kWakeupJitter / 2, mapping);
        std::optional<double> error;
        if (const auto ahead = timeline.framesAhead(heard, read)) {
            error = *ahead + resampler.inputDistance();
        }
        const auto step = loop.update(tD, error, written - read, resampler.inputFor(kFrames));
        read += step.trim;
        if (!step.play) {
            resampler.reset();
            continue;
        }
        if (!started) {
            started = true;
            result.startedAt = wokeAt;
        }

        // Ground truth: the frame heard first in this callback, against the
        // frame the sender's capture clock says is due at that moment. The
        // smoothed stamps track the capture callbacks' mean lateness, so the
        // due frame does too.
        const double position = static_cast<double>(read) - resampler.inputDistance();
        const double due = (heardAt - kLatency - kStart - kCaptureJitter / 2) * trueSenderRate;

        resampler.setRatio(step.ratio);
        const std::size_t need = resampler.inputFor(kFrames);
        if (written - read < need) {
            ++result.underruns;
            loop.restart();
            resampler.reset();
            continue;
        }
        read += need;
        resampler.process(in.data(), need, out.data(), kFrames);

        if (wokeAt > seconds - 60.0) {
            sumCorrection += loop.correction();
            ++counted;
            result.maxAbsError = std::max(result.maxAbsError, std::abs(loop.error()));
            result.maxAbsTruth = std::max(result.maxAbsTruth, std::abs(position - due));
            sumTruth += position - due;
            sumTruth2 += (position - due) * (position - due);
        }
    }
    result.meanCorrection = sumCorrection / counted;
    result.meanTruth = sumTruth / counted;
    result.sdTruth = std::sqrt(sumTruth2 / counted - result.meanTruth * result.meanTruth);
    return result;
}

}  // namespace

TEST_CASE("closed loop: plays every frame at its presentation time", "[rate_loop][simulation]") {
    for (const SimCase c : {SimCase{.ppm = 100},
                            SimCase{.ppm = -150, .skew = 40e-6, .offset = 1234.5},
                            SimCase{.ppm = 0, .skew = -60e-6, .offset = -50.0, .networkDelay = 0.03},
                            SimCase{.senderRate = 44100, .ppm = 50, .skew = 20e-6, .offset = 7.0},
                            SimCase{.deviceRate = 44100, .ppm = -80, .offset = 3.0}}) {
        const SimResult r = simulate(c, 180.0);
        // The device's rate in master time is off by the skew too.
        const double expectedPpm = ((1.0 + c.ppm * 1e-6) / (1.0 + c.skew) - 1.0) * 1e6;
        INFO(c.senderRate << " -> " << c.deviceRate << " Hz, sender " << c.ppm << " ppm, receiver skew "
             << c.skew * 1e6 << " ppm, network " << c.networkDelay * 1e3 << " ms: correction "
             << r.meanCorrection * 1e6 << " ppm (expected " << expectedPpm << "), max |error| "
             << r.maxAbsError << " frames, max |heard - due| " << r.maxAbsTruth << " frames (mean " << r.meanTruth << ", sd " << r.sdTruth << "), underruns "
             << r.underruns << ", started at " << r.startedAt << " s");
        REQUIRE(r.underruns == 0);
        REQUIRE(r.startedAt > 1.0 + 0.1);       // not before the first frame is due
        REQUIRE(r.startedAt < 1.0 + 0.1 + 0.05);
        REQUIRE(r.meanCorrection * 1e6 == Approx(expectedPpm).margin(5.0));
        // What is left is the smoothing filters' noise on the capture and
        // wakeup jitter, measured at 0.75 to 1.25 frames rms. The sender's
        // share of it is the same on every receiver, so it shifts all of
        // them together.
        REQUIRE(std::abs(r.meanTruth) < 0.6);
        REQUIRE(r.sdTruth < 1.5);
        REQUIRE(r.maxAbsTruth < 6.0);
        REQUIRE(r.maxAbsError < 4.0);
    }
}
