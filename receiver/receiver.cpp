
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

struct ReceiveStats {
    std::uint64_t discontinuities = 0;
    std::uint64_t ringDrops = 0;        // packets the ring had no room for
    std::uint64_t lostFrames = 0;       // frames the sender sent that never arrived
    std::uint64_t concealFailures = 0;  // holes the ring was too full to patch
    std::uint64_t resyncs = 0;          // holes too large to patch at all
    std::uint64_t sizeMismatches = 0;   // payloads that are not frames * bytesPerFrame
    std::atomic<std::uint64_t> underrunFrames{0};
};

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
    ReceiveStats* stats = nullptr;
};

struct PacketHeader {
    std::uint32_t seq = 0;
    std::uint32_t frames = 0;
    std::uint64_t ts = 0;

};

PacketHeader parsePacketHeader(const std::uint8_t* buf) {
    PacketHeader header;
    std::memcpy(&header.seq, buf, 4);
    std::memcpy(&header.frames, buf + 4, 4);
    std::memcpy(&header.ts, buf + 8, 8);
    return header;
}

void printReceiveStats(const ReceiveStats& stats) {
    std::cerr << "discontinuities " << stats.discontinuities
              << ", lost frames " << stats.lostFrames
              << ", conceal failures " << stats.concealFailures
              << ", resyncs " << stats.resyncs
              << ", ring drops " << stats.ringDrops
              << ", size mismatches " << stats.sizeMismatches
              << ", underrun frames " << stats.underrunFrames.load()
              << "\n";

}

static void onPlayback(void* user, void* output, ma_uint32 frameCount) {
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

    auto* const out = static_cast<std::uint8_t*>(output);
    const std::size_t got = s->ring->read(out, frameCount);
    if (got < frameCount) {
        const std::size_t short_ = frameCount - got;
        std::memset(out + got * s->bytesPerFrame, 0, short_ * s->bytesPerFrame);
        s->stats->underrunFrames.fetch_add(short_, std::memory_order_relaxed);
    }
}

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

    AudioPlayer::Config playerConfig {
        .format = ma_format_s16,
        .channels = 0,     // native
        .sampleRate = senderSampleRate,
        .periodSizeInFrames = periodSizeInFrames,
        .periods = kNumPeriods,
    };

    Ring frameRing;
    ReceiveStats stats;

    const auto origin  = Clock::now();

    PlaybackState state{
        .origin = origin,
        .sampleRate = senderSampleRate,
        .periodSizeInSamples = periodSizeInFrames,
        .stats = &stats,
    };

    AudioPlayer player;
    if (!player.open(audio, audio.playbackInfo(outputDeviceIndex).id, playerConfig, onPlayback, &state)) {
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

                const auto header = parsePacketHeader(buf.data());

                if (discard > 0) {
                    --discard;
                    expectedSeq = header.seq + 1;
                    arm();
                    return;
                }

                const std::uint32_t gap = header.seq - expectedSeq;   // unsigned, wrap-safe
                if (gap != 0) {
                    ++stats.discontinuities;
                    const std::size_t missing = static_cast<std::size_t>(gap) * header.frames;

                    if (missing > kRingFrames) {
                        ++stats.resyncs;
                        filter.invalidate();
                    } else {
                        stats.lostFrames += missing;
                        filter.skip(gap);
                        if (!insertSilence(missing)) {
                            ++stats.concealFailures;
                        }
                    }
                }
                expectedSeq = header.seq + 1;

                if (n - kHeaderBytes != header.frames * bytesPerFrame) {
                    if (stats.sizeMismatches == 0) {
                        std::cerr << "payload is " << (n - kHeaderBytes) << " bytes for "
                                  << header.frames << " frames, but this device wants "
                                  << bytesPerFrame << " bytes per frame\n";
                    }
                    ++stats.sizeMismatches;
                } else if (!frameRing.write(buf.data() + kHeaderBytes, header.frames)) {
                    ++stats.ringDrops;                // whole packet or nothing
                }

                arm();

                const double t = seconds(arrival);

                if (!configured) {
                    filter.configure(kBandwidth, header.frames, senderSampleRate);
                    filter.reset(t);
                    configured = true;
                    std::cerr << "filtering " << header.frames << " frames/callback at nominal "
                              << senderSampleRate << " Hz\n";
                    return;
                }

                if (header.frames != filter.framesPerPeriod()) {
                    std::cerr << "frame count changed " << filter.framesPerPeriod() << " -> " << header.frames
                              << ", resyncing\n";
                    filter.configure(kBandwidth, header.frames, senderSampleRate);
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

    printReceiveStats(stats);

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
