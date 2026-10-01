#include "rate_loop.h"
#include "resampler.h"
#include "time_filter.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

using Catch::Approx;

namespace {

constexpr std::size_t kFrames = 240;    // per callback and per packet

RateLoop::Config makeConfig() {
    RateLoop::Config config;
    config.nominalRatio = 1.0;
    config.deviceRate = 48000.0;
    config.framesPerCallback = kFrames;
    return config;
}

// Two reports 5 ms apart, one packet each. With no margin and no resampler
// delay the target is one callback plus one packet: 480 frames.
constexpr double kT0 = 1.000;
constexpr double kT1 = 1.005;
constexpr std::uint64_t kK0 = 10000;
constexpr std::uint64_t kK1 = kK0 + kFrames;
constexpr double kTarget = 480.0;

// Halfway between the reports d_N is half a packet, so this read position
// makes the error exactly zero.
constexpr double kTMid = 1.0025;
constexpr std::uint64_t kKBalanced = kK0 + 120 - 480;

RateLoop makeLoop(RateLoop::Config config = makeConfig()) {
    RateLoop loop;
    loop.configure(config);
    loop.addReport({kT0, kK0, false});
    loop.addReport({kT1, kK1, false});
    return loop;
}

}  // namespace

TEST_CASE("no step before two reports", "[rate_loop]") {
    RateLoop loop;
    loop.configure(makeConfig());
    REQUIRE_FALSE(loop.update(kTMid, 0, 0.0, 0).play);

    loop.addReport({kT0, kK0, false});
    REQUIRE_FALSE(loop.update(kTMid, 0, 0.0, 0).play);

    loop.addReport({kT1, kK1, false});
    REQUIRE(loop.update(kTMid, kKBalanced, 0.0, 1000).play);
}

TEST_CASE("the target covers a callback, a packet, the resampler and the margin", "[rate_loop]") {
    auto config = makeConfig();
    config.nominalRatio = 44100.0 / 48000.0;
    config.resamplerDelay = 32.0;
    config.margin = 96.0;
    const RateLoop loop = makeLoop(config);
    REQUIRE(loop.target() == Approx(44100.0 / 48000.0 * 240 + 240 + 32 + 96));
}

TEST_CASE("the error follows the paper's equation (2)", "[rate_loop]") {
    RateLoop loop = makeLoop();
    // [k_N0 - k_D] = 100, d_N = 120, d_res = 0.5, target 480
    const auto step = loop.update(kTMid, kK0 - 100, 0.5, 1000);
    REQUIRE(loop.error() == Approx(100 + 120 + 0.5 - kTarget));
    REQUIRE_FALSE(step.play);
}

TEST_CASE("the error interpolates between the two reports", "[rate_loop]") {
    for (const double fraction : {0.0, 0.25, 0.9, 1.0}) {
        RateLoop loop = makeLoop();
        (void)loop.update(kT0 + fraction * (kT1 - kT0), kK0, 0.0, 0);
        REQUIRE(loop.error() == Approx(fraction * kFrames - kTarget));
    }
}

TEST_CASE("frame counters may wrap", "[rate_loop]") {
    constexpr std::uint64_t kNearEnd = std::numeric_limits<std::uint64_t>::max() - 100;
    RateLoop loop;
    loop.configure(makeConfig());
    loop.addReport({kT0, kNearEnd, false});
    loop.addReport({kT1, kNearEnd + kFrames, false});    // wraps
    (void)loop.update(kTMid, kNearEnd - 500, 0.0, 0);
    REQUIRE(loop.error() == Approx(500 + 120 - kTarget));
}

TEST_CASE("waits while too little is buffered", "[rate_loop]") {
    RateLoop loop = makeLoop();
    const auto step = loop.update(kTMid, kKBalanced + 1, 0.0, 1000);
    REQUIRE_FALSE(step.play);
    REQUIRE(step.trim == 0);
    REQUIRE(loop.phase() == RateLoop::Phase::Waiting);
}

TEST_CASE("starts by trimming the excess", "[rate_loop]") {
    RateLoop loop = makeLoop();
    const auto step = loop.update(kTMid, kKBalanced - 10, 0.4, 1000);
    REQUIRE(step.play);
    REQUIRE(step.trim == 10);
    REQUIRE(loop.error() == Approx(0.4));
    REQUIRE(loop.phase() == RateLoop::Phase::Settling);
}

TEST_CASE("never trims more than the ring holds", "[rate_loop]") {
    RateLoop loop = makeLoop();
    const auto step = loop.update(kTMid, kKBalanced - 400, 0.0, 300);
    REQUIRE(step.play);
    REQUIRE(step.trim == 300);
    REQUIRE(loop.error() == Approx(100.0));
}

TEST_CASE("does not start on reports alone", "[rate_loop]") {
    // The reports say plenty has arrived, but the ring holds less than the
    // target minus a packet: the stream stopped and the line extrapolates.
    RateLoop loop = makeLoop();
    const auto step = loop.update(kTMid + 1.0, kKBalanced, 0.0, 239);
    REQUIRE(loop.error() > 0.0);
    REQUIRE_FALSE(step.play);
    REQUIRE(loop.phase() == RateLoop::Phase::Waiting);

    REQUIRE(loop.update(kTMid + 1.0, kKBalanced, 0.0, 240).play);
}

TEST_CASE("too much buffered reads faster, too little slower", "[rate_loop]") {
    SECTION("too much") {
        RateLoop loop = makeLoop();
        REQUIRE(loop.update(kTMid, kKBalanced, 0.0, 1000).play);
        const auto step = loop.update(kTMid, kKBalanced - 50, 0.0, 1000);
        REQUIRE(step.ratio > 1.0);
    }
    SECTION("too little") {
        RateLoop loop = makeLoop();
        REQUIRE(loop.update(kTMid, kKBalanced, 0.0, 1000).play);
        const auto step = loop.update(kTMid, kKBalanced + 50, 0.0, 1000);
        REQUIRE(step.play);     // once started, a shortfall is the loop's to fix
        REQUIRE(step.ratio < 1.0);
    }
}

TEST_CASE("the ratio is relative to the nominal ratio", "[rate_loop]") {
    auto config = makeConfig();
    config.nominalRatio = 44100.0 / 48000.0;
    RateLoop loop;
    loop.configure(config);
    loop.addReport({kT0, kK0, false});
    loop.addReport({kT1, kK1, false});
    const std::uint64_t balanced = kK0 + 120 - static_cast<std::uint64_t>(std::lround(loop.target()));
    const auto step = loop.update(kTMid, balanced, 0.0, 1000);
    REQUIRE(step.play);
    REQUIRE(step.ratio == Approx(44100.0 / 48000.0).margin(1e-5));   // what is left is a rounded half frame
}

TEST_CASE("the correction is limited", "[rate_loop]") {
    RateLoop loop = makeLoop();
    REQUIRE(loop.update(kTMid, kKBalanced, 0.0, 1000).play);
    RateLoop::Step step;
    for (int i = 0; i < 1000; ++i) {
        step = loop.update(kTMid, kKBalanced - 5000, 0.0, 10000);
    }
    REQUIRE(step.ratio == Approx(1.005));
}

TEST_CASE("settling ends after the start time", "[rate_loop]") {
    RateLoop loop = makeLoop();
    REQUIRE(loop.update(kTMid, kKBalanced, 0.0, 1000).play);
    REQUIRE(loop.phase() == RateLoop::Phase::Settling);
    (void)loop.update(kTMid + 3.9, kKBalanced, 0.0, 1000);
    REQUIRE(loop.phase() == RateLoop::Phase::Settling);
    (void)loop.update(kTMid + 4.0, kKBalanced, 0.0, 1000);
    REQUIRE(loop.phase() == RateLoop::Phase::Running);
}

TEST_CASE("a restart report waits for two new reports", "[rate_loop]") {
    RateLoop loop = makeLoop();
    REQUIRE(loop.update(kTMid, kKBalanced, 0.0, 1000).play);

    loop.addReport({2.0, 50000, true});
    REQUIRE(loop.phase() == RateLoop::Phase::Waiting);
    REQUIRE_FALSE(loop.update(2.001, 50000, 0.0, 0).play);

    loop.addReport({2.005, 50000 + kFrames, false});
    const auto step = loop.update(2.001, 49000, 0.0, 1000);
    REQUIRE(step.play);
    REQUIRE(step.trim == 1000 + 48 - 480);   // the error at the start, from the new reports only
}

TEST_CASE("a restart keeps the learned correction", "[rate_loop]") {
    RateLoop loop = makeLoop();
    REQUIRE(loop.update(kTMid, kKBalanced, 0.0, 1000).play);
    for (int i = 0; i < 200; ++i) {
        (void)loop.update(kTMid, kKBalanced - 20, 0.0, 1000);
    }
    const double learned = loop.correction();
    REQUIRE(learned > 0.0);

    loop.restart();
    loop.addReport({kT0, kK0, false});
    loop.addReport({kT1, kK1, false});
    const auto step = loop.update(kTMid, kKBalanced, 0.0, 1000);
    REQUIRE(step.play);
    REQUIRE(step.ratio > 1.0);
    REQUIRE(loop.correction() < learned);   // the proportional part is gone, the integral stays
}

// The whole receiver in miniature: a sender whose clock is off by `ppm`,
// packets arriving with network jitter, a device calling back with wakeup
// jitter, both sides smoothed by TimeFilters, and the loop steering a real
// resampler. Only frame counts move; the audio is silence.
namespace {

struct SimResult {
    int underruns = 0;
    double meanCorrection = 0.0;    // over the last 60 s
    double maxAbsError = 0.0;       // over the last 60 s, frames
    std::uint64_t maxFill = 0;
};

SimResult simulate(double senderRate, double deviceRate, double ppm, double seconds) {
    constexpr double kBandwidth = 0.05;
    constexpr double kNetworkJitter = 0.002;    // arrival delay spread, seconds
    constexpr double kWakeupJitter = 0.0003;
    constexpr double kMargin = 0.003;           // seconds

    std::mt19937 rng{42};
    std::uniform_real_distribution<double> network{0.0, kNetworkJitter};
    std::uniform_real_distribution<double> wakeup{0.0, kWakeupJitter};

    const double nominal = senderRate / deviceRate;
    const double trueSenderRate = senderRate * (1.0 + ppm * 1e-6);

    Resampler resampler;
    resampler.configure(1, nominal, 8);

    RateLoop loop;
    RateLoop::Config config;
    config.nominalRatio = nominal;
    config.deviceRate = deviceRate;
    config.framesPerCallback = kFrames;
    config.resamplerDelay = static_cast<double>(resampler.halfLength());
    config.margin = kMargin * senderRate;
    loop.configure(config);

    TimeFilter net, dev;
    net.configure(kBandwidth, kFrames, senderRate);
    dev.configure(kBandwidth, kFrames, deviceRate);

    std::uint64_t written = 0, read = 0;
    std::vector<std::int16_t> in(4096), out(kFrames);
    SimResult result;
    double sumCorrection = 0.0;
    int counted = 0;

    std::uint64_t packet = 0, callback = 0;
    for (;;) {
        const double arrival = 1.0 + static_cast<double>(packet * kFrames) / trueSenderRate + network(rng);
        const double wake = 1.0 + static_cast<double>(callback * kFrames) / deviceRate + wakeup(rng);
        if (std::min(arrival, wake) > seconds) {
            break;
        }

        if (arrival < wake) {
            written += kFrames;
            if (net.ready()) {
                net.update(arrival);
            } else {
                net.reset(arrival);
            }
            loop.addReport({net.nextTime(), written + kFrames, false});
            ++packet;
            continue;
        }

        ++callback;
        if (dev.ready()) {
            dev.update(wake);
        } else {
            dev.reset(wake);
        }
        const auto step = loop.update(dev.time(), read, resampler.inputDistance(), written - read);
        if (!step.play) {
            resampler.reset();
            continue;
        }
        read += step.trim;
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

        result.maxFill = std::max(result.maxFill, written - read + need);
        if (wake > seconds - 60.0) {
            sumCorrection += loop.correction();
            ++counted;
            result.maxAbsError = std::max(result.maxAbsError, std::abs(loop.error()));
        }
    }
    result.meanCorrection = sumCorrection / counted;
    return result;
}

}  // namespace

TEST_CASE("closed loop: locks onto the sender's clock", "[rate_loop][simulation]") {
    struct Case { double senderRate, deviceRate, ppm; };
    for (const Case c : {Case{48000, 48000, 100}, Case{48000, 48000, -150}, Case{48000, 48000, 0},
                         Case{44100, 48000, 50}, Case{48000, 44100, -80}}) {
        const SimResult r = simulate(c.senderRate, c.deviceRate, c.ppm, 180.0);
        INFO(c.senderRate << " -> " << c.deviceRate << " Hz at " << c.ppm << " ppm: correction "
             << r.meanCorrection * 1e6 << " ppm, max |error| " << r.maxAbsError
             << " frames, max fill " << r.maxFill << ", underruns " << r.underruns);
        REQUIRE(r.underruns == 0);
        // Network jitter the DLL does not remove makes the correction wander
        // slowly around the true value, by some 15 ppm over 20 s.
        REQUIRE(r.meanCorrection * 1e6 == Approx(c.ppm).margin(10.0));
        REQUIRE(r.maxAbsError < 20.0);
        REQUIRE(r.maxFill < 2048);      // the receiver's ring
    }
}
