#define MA_IMPLEMENTATION
#include <miniaudio.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <thread>

#include "spsc_ring.h"
#include "time_filter.h"

using Clock = std::chrono::steady_clock;

namespace {

constexpr double kBandwidth = 0.05;
constexpr ma_uint32 kPeriodRequest = 240;

std::atomic<bool> g_running{true};
void on_sigint(int) {
    g_running.store(false, std::memory_order_relaxed);
}

struct Event {
    Clock::time_point wake;
    ma_uint32 frames;
};

struct Context {
    SpscRing<Event, 1024> queue;
    std::atomic<bool> dropped{false};
};

void data_callback(ma_device* device, void* output, const void* /*input*/, ma_uint32 frameCount) {
    const auto wake = Clock::now();
    auto* ctx = static_cast<Context*>(device->pUserData);

    if (!ctx->queue.push(Event{wake, frameCount})) {
        ctx->dropped.store(true, std::memory_order_relaxed);
    }

    (void)output;
}

}

int main() {
    std::signal(SIGINT, on_sigint);

    Context ctx;

    ma_device_config config   = ma_device_config_init(ma_device_type_playback);
    config.playback.format    = ma_format_s16;
    config.playback.channels  = 0;  // native — avoid a channel converter
    config.sampleRate         = 0;
    config.periodSizeInFrames = kPeriodRequest;
    config.periods            = 3;
    config.performanceProfile = ma_performance_profile_low_latency;
    config.dataCallback       = data_callback;
    config.pUserData          = &ctx;

    ma_device device;
    ma_result r = ma_device_init(nullptr, &config, &device);
    if (r != MA_SUCCESS) {
        std::fprintf(stderr, "ma_device_init: %s\n", ma_result_description(r));
        return 1;
    }

    const double nominalRate = static_cast<double>(device.sampleRate);

    std::fprintf(stderr, "device           : %s\n", device.playback.name);
    std::fprintf(stderr, "callback rate    : %u Hz\n", device.sampleRate);
    std::fprintf(stderr, "internal rate    : %u Hz\n", device.playback.internalSampleRate);
    std::fprintf(stderr, "internal period  : %u frames\n", device.playback.internalPeriodSizeInFrames);
    std::fprintf(stderr, "fixed-size cb    : %s\n", device.noFixedSizedCallback ? "no" : "yes");

    if (device.playback.internalSampleRate != device.sampleRate) {
        std::fprintf(stderr, "WARNING: a resampler is in the path; the measured rate will not be the hardware crystal.\n");
    }

    if ((r = ma_device_start(&device)) != MA_SUCCESS) {
        std::fprintf(stderr, "ma_device_start: %s\n", ma_result_description(r));
        ma_device_uninit(&device);
        return 1;
    }

    const auto origin  = Clock::now();
    const auto seconds = [origin](Clock::time_point tp) {
        return std::chrono::duration<double>(tp - origin).count();
    };

    TimeFilter filter;
    bool configured = false;
    int discard = 10;

    std::printf("# frame  filtered_time  error  period  rate\n");

    while (g_running.load(std::memory_order_relaxed)) {
        Event ev;
        if (!ctx.queue.pop(ev)) {
            // Timestamps are taken in the callback, so polling latency here
            // does not affect the estimate at all.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (discard > 0) {
            --discard;
            continue;
        }

        const double t = seconds(ev.wake);

        if (!configured) {
            filter.configure(kBandwidth, ev.frames, nominalRate);
            filter.reset(t);
            configured = true;
            std::fprintf(stderr, "filtering %u frames/callback at nominal %.0f Hz\n", ev.frames, nominalRate);
            continue;
        }

        if (ctx.dropped.exchange(false, std::memory_order_relaxed)) {
            std::fprintf(stderr, "queue overflow — events lost, resyncing\n");
            filter.invalidate();
        }

        if (ev.frames != filter.framesPerPeriod()) {
            std::fprintf(stderr, "frame count changed %u -> %u, resyncing\n", filter.framesPerPeriod(), ev.frames);
            filter.configure(kBandwidth, ev.frames, nominalRate);
            filter.invalidate();
        }

        if (!filter.ready()) {
            filter.reset(t);
            continue;
        }

        filter.update(t);

        std::printf("%llu %.9f %.9f %.12f %.4f\n", static_cast<unsigned long long>(filter.frame()), filter.time(), filter.error(), filter.period(), filter.rate());
    }

    std::fprintf(stderr, "\nstopping\n");
    ma_device_stop(&device);
    ma_device_uninit(&device);
    return 0;
}