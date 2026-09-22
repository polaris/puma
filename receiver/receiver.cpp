
#include <asio.hpp>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>
#include <CLI/CLI.hpp>

// time_filter.h pulls in the miniaudio declarations; the implementation block
// sits outside that header's include guard, so this has to be compiled here.
#define MA_IMPLEMENTATION
#include <miniaudio.h>

#include "audio_context.h"
#include "audio_player.h"
#include "netint.h"
#include "time_filter.h"
#include "frame_ring.h"

using asio::ip::udp;
using Clock = std::chrono::steady_clock;

constexpr double kBandwidth = 0.05;
constexpr std::string_view kDefaultMulticastGroup = "239.255.0.1";
constexpr unsigned short kDefaultMulticastPort = 12345;
constexpr unsigned int kDefaultSampleRate = 48000;
constexpr unsigned int kPeriodSizeInFrames = 240;
constexpr unsigned int kNumPeriods = 3;
constexpr std::size_t kHeaderBytes = 16;        // seq(4) + frames(4) + timestamp(8)

constexpr std::size_t kRingFrames = 2048;
using Ring = FrameRing<kRingFrames>;

void enumerateNetworkInterfaces();
void enumerateOutputDevices(const AudioContext& audio);

struct PlaybackState {
    std::chrono::steady_clock::time_point origin;
    bool configured = false;
    TimeFilter timeFilter{};
    unsigned int sampleRate = 0;
    unsigned int periodSizeInSamples = 0;

    // Filled in after the device is open and before it is started, because the
    // audio thread reads them and does not exist until start().
    Ring* ring = nullptr;
    std::size_t bytesPerFrame = 0;

    // Written on the audio thread, read on shutdown.
    std::atomic<std::uint64_t> underrunFrames{0};
};

int main(int argc, char** argv) {
    CLI::App app{"Receiver"};
    argv = app.ensure_utf8(argv);

    AudioContext audio;
    if (!audio.init()) {
        std::cerr << "Failed to initialise the audio context\n";
        return 2;
    }
    if (audio.playbackCount() == 0) {
        std::cerr << "No audio playback devices available\n";
        return 2;
    }

    app.add_flag("--enumOutputDevices",
        [&audio] (int64_t) {
            enumerateOutputDevices(audio);
            throw CLI::Success();
        }, "Enumerate audio output devices")
        ->trigger_on_parse();
    app.add_flag("--enumNetworkInterfaces",
        [] (int64_t) {
            enumerateNetworkInterfaces();
            throw CLI::Success();
        }, "Enumerate network interfaces")
        ->trigger_on_parse();

    unsigned int outputDeviceIndex = 0;
    app.add_option("-o,--outputDeviceIndex", outputDeviceIndex, "Index of the audio output device")
        ->required();
    
    unsigned int senderSampleRate = kDefaultSampleRate;
    app.add_option("-s,--senderSampleRate", senderSampleRate, "Sender sample rate")
        ->check(CLI::PositiveNumber);

    unsigned int periodSizeInFrames = kPeriodSizeInFrames;
    app.add_option("-f,--periodSizeInFrames", periodSizeInFrames, "Period size in frames")
        ->check(CLI::PositiveNumber);

    std::string networkInterface;
    app.add_option("-n,--networkInterface", networkInterface, "Network interface");

    std::string multicastGroup{kDefaultMulticastGroup};
    app.add_option("-m,--multicastGroup", multicastGroup, "Multicast group address")
        ->check(CLI::ValidIPV4);
    
    unsigned short multicastPort = kDefaultMulticastPort;
    app.add_option("-p,--multicastPort", multicastPort, "Multicast port");
    
    CLI11_PARSE(app, argc, argv);

    if (outputDeviceIndex >= audio.playbackCount()) {
        std::cerr << "Output device with index " << outputDeviceIndex << " not available\n";
        return 2;
    }

    const auto chosen = !networkInterface.empty() ? net::find(networkInterface) : net::selectDefault();
    if (!chosen) {
        std::cerr << (!networkInterface.empty() ? "No such interface\n" : "Ambiguous or none; name one explicitly\n");
        return 1;
    }
    std::cout << "Using network interface " << chosen->name << " " << chosen->address.to_string() << "\n";

    const asio::ip::udp::endpoint group(asio::ip::make_address(multicastGroup), multicastPort);
    asio::io_context io;
    asio::ip::udp::socket rx(io);
    net::configureReceiver(rx, group, *chosen);
    std::cout << "Receiver configured\n";

    AudioPlayer::Config playerConfig;
    playerConfig.format = ma_format_s16;
    playerConfig.channels = 0;     // native
    playerConfig.sampleRate = senderSampleRate;
    playerConfig.periodSizeInFrames = periodSizeInFrames;
    playerConfig.periods = kNumPeriods;

    Ring frameRing;

    const auto origin  = Clock::now();

    PlaybackState state{
        .origin = origin,
        .sampleRate = senderSampleRate,
        .periodSizeInSamples = periodSizeInFrames,
    };

    AudioPlayer player;
    if (!player.open(audio, audio.playbackInfo(outputDeviceIndex).id, playerConfig, 
        [](void* user, void* output, ma_uint32 frameCount) {
            const auto call = Clock::now();
            auto* const s = static_cast<PlaybackState*>(user);
            const auto t = std::chrono::duration<double>(call - s->origin).count();
            if (!s->configured) {
                s->timeFilter.configure(kBandwidth, s->periodSizeInSamples, s->sampleRate);
                s->timeFilter.reset(t);
                s->configured = true;
            } else {
                s->timeFilter.update(t);
            }

            // The buffer has to be filled on every call, including the first:
            // returning early would hand the device whatever was in it.
            auto* const out = static_cast<std::uint8_t*>(output);
            const std::size_t got = s->ring->read(out, frameCount);
            if (got < frameCount) {
                const std::size_t short_ = frameCount - got;
                std::memset(out + got * s->bytesPerFrame, 0, short_ * s->bytesPerFrame);
                s->underrunFrames.fetch_add(short_, std::memory_order_relaxed);
            }
        }, &state)) {
        std::cerr << "Failed to open the selected output device\n";
        return 2;
    }
    if (player.internalSampleRate() != senderSampleRate) {
        std::cerr << "Output device runs at " << player.internalSampleRate()
                  << " Hz, but the sender uses " << senderSampleRate << " Hz\n";
        return 2;
    }
    std::cout << "playback: " << player.channels() << " ch s16 @ " << player.sampleRate() << " Hz\n";

    // Everything the audio callback touches has to be in place before start().
    const std::size_t bytesPerFrame = player.bytesPerFrame();
    if (!frameRing.init(bytesPerFrame)) {
        std::cerr << "Unsupported frame size: " << bytesPerFrame << " bytes\n";
        return 2;
    }
    state.ring = &frameRing;
    state.bytesPerFrame = bytesPerFrame;

    const auto seconds = [origin](Clock::time_point tp) {
        return std::chrono::duration<double>(tp - origin).count();
    };

    std::array<std::uint8_t, 8192> buf;
    TimeFilter filter;
    bool configured = false;
    int discard = 10;

    std::uint32_t expectedSeq = 0;
    std::uint64_t discontinuities = 0;
    std::uint64_t ringDrops = 0;        // packets the ring had no room for
    std::uint64_t lostFrames = 0;       // frames the sender sent that never arrived
    std::uint64_t concealFailures = 0;  // holes the ring was too full to patch
    std::uint64_t resyncs = 0;          // holes too large to patch at all
    std::uint64_t sizeMismatches = 0;   // payloads that are not frames * bytesPerFrame
    bool stopping = false;      // io thread only, set by the teardown below

    const auto insertSilence = [&](std::size_t missing) {
        const Regions regions = frameRing.acquireWrite(missing);
        if (regions.frames() < missing) {
            return false;
        }
        for (const Region* part : {&regions.region1(), &regions.region2()}) {
            if (part->len > 0) {
                std::memset(part->buf, 0, part->len * bytesPerFrame);
            }
        }
        return frameRing.commitWrite(missing);
    };

    std::function<void()> arm = [&] {
        rx.async_receive(asio::buffer(buf),
            [&](const asio::error_code& ec, std::size_t n) {
                if (stopping || ec == asio::error::operation_aborted) {
                    return;
                }
                if (ec) { 
                    arm();
                    std::cerr << "receive: " << ec.message() << "\n";
                    return;
                }                

                if (n < 16) {
                    arm();
                    std::cerr << "packet too small\n";
                    return;
                }

                const auto arrival = Clock::now();

                std::uint32_t seq;
                std::memcpy(&seq, buf.data() + 0, 4);

                if (discard > 0) {
                    --discard;
                    expectedSeq = seq + 1;
                    arm();
                    return;
                }

                ma_uint32 frames;
                std::memcpy(&frames, buf.data() + 4, 4);
                std::uint64_t ts;
                std::memcpy(&ts, buf.data() + 8, 8);

                arm();

                const std::uint32_t gap = seq - expectedSeq;   // unsigned, wrap-safe
                if (gap != 0) {
                    ++discontinuities;
                    const std::size_t missing = static_cast<std::size_t>(gap) * frames;

                    if (missing > kRingFrames) {
                        ++resyncs;
                        filter.invalidate();
                    } else {
                        lostFrames += missing;
                        filter.skip(gap);
                        if (!insertSilence(missing)) {
                            ++concealFailures;
                        }
                    }
                }
                expectedSeq = seq + 1;

                if (n - kHeaderBytes != frames * bytesPerFrame) {
                    if (sizeMismatches == 0) {
                        std::cerr << "payload is " << (n - kHeaderBytes) << " bytes for "
                                  << frames << " frames, but this device wants "
                                  << bytesPerFrame << " bytes per frame\n";
                    }
                    ++sizeMismatches;
                } else if (!frameRing.write(buf.data() + kHeaderBytes, frames)) {
                    ++ringDrops;                // whole packet or nothing
                }

                const double t = seconds(arrival);

                if (!configured) {
                    filter.configure(kBandwidth, frames, senderSampleRate);
                    filter.reset(t);
                    configured = true;
                    std::cerr << "filtering " << frames << " frames/callback at nominal "
                              << senderSampleRate << " Hz\n";
                    return;
                }

                if (frames != filter.framesPerPeriod()) {
                    std::cerr << "frame count changed " << filter.framesPerPeriod() << " -> " << frames
                              << ", resyncing\n";
                    filter.configure(kBandwidth, frames, senderSampleRate);
                    filter.invalidate();
                }

                if (!filter.ready()) {
                    filter.reset(t);
                    return;
                }

                filter.update(t);

                std::cout << filter.frame() << ' ' << std::fixed
                          << std::setprecision(9) << filter.time() << ' '
                          << std::setprecision(9) << filter.error() << ' '
                          << std::setprecision(12) << filter.period() << ' '
                          << std::setprecision(4) << filter.rate() << '\n';
            });
    };

    if (!player.start()) {
        std::cerr << "Failed to start the playback device\n";
        return 2;
    }

    arm();
    std::thread worker([&]{ io.run(); });

    asio::io_context wait;
    asio::signal_set signals{wait, SIGINT, SIGTERM};
    signals.async_wait([&](auto, int) {
        std::cerr << "\nshutting down\n";
        wait.stop();
    });

    wait.run();

    player.stop();

    // Close from inside the io thread, and latch the flag first so nothing
    // re-arms afterwards. close() here is the non-throwing overload: an
    // exception escaping a handler would take down io.run().
    asio::post(io, [&]{
        stopping = true;
        asio::error_code ignored;
        rx.close(ignored);
    });
    worker.join();

    std::cerr << "discontinuities " << discontinuities
              << ", lost frames " << lostFrames
              << ", conceal failures " << concealFailures
              << ", resyncs " << resyncs
              << ", ring drops " << ringDrops
              << ", size mismatches " << sizeMismatches
              << ", underrun frames " << state.underrunFrames.load()
              << "\n";

    return 0;
}

void enumerateOutputDevices(const AudioContext& audio) {
    for (ma_uint32 deviceIndex = 0; deviceIndex < audio.playbackCount(); deviceIndex += 1) {
        const ma_device_info& info = audio.playbackInfo(deviceIndex);
        std::cout << deviceIndex << " - " << info.name
                  << (info.isDefault ? " (default)" : "") << "\n";
    }
}

void enumerateNetworkInterfaces() {
    const auto all = net::enumerate();
    std::cout << "interfaces:\n";
    for (const auto &i : all) {
        std::cout << "  " << i.name << "  " << i.address.to_string() << "  idx=" << i.index;
        if (!i.description.empty()) {
            std::cout << "  (" << i.description << ")";
        }
        std::cout << "\n";
    }
}
