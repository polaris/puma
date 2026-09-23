
#include <asio.hpp>
#include <array>
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
    unsigned int sampleRate = 0;
    unsigned int periodSizeInSamples = 0;
    std::size_t bytesPerFrame = 0;
    
    Ring* ring = nullptr;
    ReceiveStats* stats = nullptr;

    std::chrono::steady_clock::time_point origin;
    bool configured = false;
    TimeFilter timeFilter{};
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

class PacketReceiver {
public:
    PacketReceiver(udp::socket& rx, Ring& ring, ReceiveStats& stats, std::size_t bytesPerFrame,
                   unsigned int sampleRate, Clock::time_point origin)
    : rx_{rx}
    , ring_{ring}
    , stats_{stats}
    , bytesPerFrame_{bytesPerFrame}
    , sampleRate_{sampleRate}
    , origin_{origin} {
    }

    PacketReceiver(const PacketReceiver&) = delete;
    PacketReceiver& operator=(const PacketReceiver&) = delete;

    void start() {
        arm();
    }

    void stop() {
        stopping_ = true;
        asio::error_code ignored;
        rx_.close(ignored);
    }

private:
    void arm() {
        rx_.async_receive(asio::buffer(buf_),
            [this](const asio::error_code& ec, std::size_t n) { onReceive(ec, n); });
    }

    void onReceive(const asio::error_code& ec, std::size_t n) {
        if (stopping_ || ec == asio::error::operation_aborted) {
            return;
        }
        if (ec) {
            arm();
            std::cerr << "receive: " << ec.message() << "\n";
            return;
        }
        if (n < kHeaderBytes) {
            arm();
            std::cerr << "packet too small\n";
            return;
        }

        const auto arrival = Clock::now();
        const auto header = parsePacketHeader(buf_.data());

        if (discard_ > 0) {
            --discard_;
            expectedSeq_ = header.seq + 1;
            arm();
            return;
        }

        const std::uint32_t gap = header.seq - expectedSeq_;
        if (gap != 0) {
            concealGap(gap, header.frames);
        }
        expectedSeq_ = header.seq + 1;

        storePayload(header.frames, n - kHeaderBytes);

        arm();

        trackTiming(arrival, header.frames);
    }

    void concealGap(std::uint32_t gap, std::uint32_t frames) {
        ++stats_.discontinuities;
        const std::size_t missing = static_cast<std::size_t>(gap) * frames;

        if (missing > kRingFrames) {
            ++stats_.resyncs;
            filter_.invalidate();
            return;
        }

        stats_.lostFrames += missing;
        filter_.skip(gap);
        if (!insertSilence(missing)) {
            ++stats_.concealFailures;
        }
    }

    bool insertSilence(std::size_t missing) {
        const Regions regions = ring_.acquireWrite(missing);
        if (regions.frames() < missing) {
            return false;
        }
        for (const Region* part : {&regions.region1(), &regions.region2()}) {
            if (part->len > 0) {
                std::memset(part->buf, 0, part->len * bytesPerFrame_);
            }
        }
        return ring_.commitWrite(missing);
    }

    void storePayload(std::uint32_t frames, std::size_t payloadBytes) {
        if (payloadBytes != frames * bytesPerFrame_) {
            if (stats_.sizeMismatches == 0) {
                std::cerr << "payload is " << payloadBytes << " bytes for "
                          << frames << " frames, but this device wants "
                          << bytesPerFrame_ << " bytes per frame\n";
            }
            ++stats_.sizeMismatches;
        } else if (!ring_.write(buf_.data() + kHeaderBytes, frames)) {
            ++stats_.ringDrops;                // whole packet or nothing
        }
    }

    void trackTiming(Clock::time_point arrival, std::uint32_t frames) {
        const double t = std::chrono::duration<double>(arrival - origin_).count();

        if (!filterConfigured_) {
            filter_.configure(kBandwidth, frames, sampleRate_);
            filter_.reset(t);
            filterConfigured_ = true;
            std::cerr << "filtering " << frames << " frames/callback at nominal "
                      << sampleRate_ << " Hz\n";
            return;
        }

        if (frames != filter_.framesPerPeriod()) {
            std::cerr << "frame count changed " << filter_.framesPerPeriod() << " -> " << frames
                      << ", resyncing\n";
            filter_.configure(kBandwidth, frames, sampleRate_);
            filter_.invalidate();
        }

        if (!filter_.ready()) {
            filter_.reset(t);
            return;
        }

        filter_.update(t);

        std::cout << filter_.frame() << ' ' << std::fixed
                  << std::setprecision(9) << filter_.time() << ' '
                  << std::setprecision(9) << filter_.error() << ' '
                  << std::setprecision(12) << filter_.period() << ' '
                  << std::setprecision(4) << filter_.rate() << '\n';
    }

    udp::socket& rx_;
    Ring& ring_;
    ReceiveStats& stats_;
    const std::size_t bytesPerFrame_;
    const unsigned int sampleRate_;
    const Clock::time_point origin_;

    TimeFilter filter_;
    bool filterConfigured_ = false;
    int discard_ = 10;
    std::uint32_t expectedSeq_ = 0;
    bool stopping_ = false;

    std::array<std::uint8_t, 8192> buf_;    // last, so the hot members share cache lines
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

    PacketReceiver receiver(rx, frameRing, stats, bytesPerFrame, senderSampleRate, origin);

    if (!player.start()) {
        std::cerr << "Failed to start the playback device\n";
        return 2;
    }

    receiver.start();
    std::thread worker([&]{ io.run(); });

    asio::io_context wait;
    asio::signal_set signals{wait, SIGINT, SIGTERM};
    signals.async_wait([&](auto, int) {
        std::cerr << "\nshutting down\n";
        wait.stop();
    });

    wait.run();

    player.stop();

    // Close from inside the io thread.
    asio::post(io, [&receiver]{ receiver.stop(); });
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
