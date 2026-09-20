
#include <asio.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <CLI/CLI.hpp>

// time_filter.h pulls in the miniaudio declarations; the implementation block
// sits outside that header's include guard, so this has to be compiled here.
#define MA_IMPLEMENTATION
#include <miniaudio.h>

#include "netint.h"
#include "time_filter.h"

using asio::ip::udp;
using Clock = std::chrono::steady_clock;

constexpr double kBandwidth = 0.05;
constexpr std::string_view kDefaultMulticastGroup = "239.255.0.1";
constexpr unsigned short kDefaultMulticastPort = 12345;
constexpr unsigned int kDefaultSampleRate = 48000;
constexpr unsigned int kPeriodSizeInFrames = 240;
constexpr unsigned int kNumPeriods = 3;

void enumerateNetworkInterfaces();
void enumerateOutputDevices(ma_uint32 playbackCount, ma_device_info *playbackInfos);

// Silence for now: the received packets still go straight to the WAV file.
// Feeding them to the device is what the resampler in front of this will do.
void data_callback(ma_device* device, void* output, const void* input, ma_uint32 frameCount) {
    (void)input;
    ma_silence_pcm_frames(output, frameCount, device->playback.format, device->playback.channels);
}

int main(int argc, char** argv) {
    CLI::App app{"Receiver"};
    argv = app.ensure_utf8(argv);

    ma_context context;
    if (ma_context_init(nullptr, 0, nullptr, &context) != MA_SUCCESS) {
        std::cerr << "Failed to initialise the audio context\n";
        return 2;
    }

    ma_device_info* playbackInfos = nullptr;
    ma_uint32 playbackDeviceCount = 0;
    ma_device_info* captureInfos = nullptr;
    ma_uint32 captureDeviceCount = 0;
    if (ma_context_get_devices(&context, &playbackInfos, &playbackDeviceCount, &captureInfos, &captureDeviceCount) != MA_SUCCESS) {
        std::cerr << "Failed to enumerate audio devices\n";
        ma_context_uninit(&context);
        return 2;
    }
    if (playbackDeviceCount == 0) {
        std::cerr << "No audio playback devices available\n";
        ma_context_uninit(&context);
        return 2;
    }

    app.add_flag("--enumOutputDevices",
        [&context, playbackDeviceCount, playbackInfos] (int64_t) {
            enumerateOutputDevices(playbackDeviceCount, playbackInfos);
            ma_context_uninit(&context);
            exit(0);
        }, "Enumerate audio output devices")
        ->trigger_on_parse();
    app.add_flag("--enumNetworkInterfaces",
        [&context] (int64_t) {
            enumerateNetworkInterfaces();
            ma_context_uninit(&context);
            exit(0);
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

    if (outputDeviceIndex >= playbackDeviceCount) {
        std::cerr << "Output device with index " << outputDeviceIndex << " not available\n";
        ma_context_uninit(&context);
        return 2;
    }

    const auto chosen = !networkInterface.empty() ? net::find(networkInterface) : net::selectDefault();
    if (!chosen) {
        std::cerr << (!networkInterface.empty() ? "No such interface\n" : "Ambiguous or none; name one explicitly\n");
        ma_context_uninit(&context);
        return 1;
    }
    std::cout << "Using network interface " << chosen->name << " " << chosen->address.to_string() << "\n";

    const asio::ip::udp::endpoint group(asio::ip::make_address(multicastGroup), multicastPort);
    asio::io_context io;
    asio::ip::udp::socket rx(io);
    net::configureReceiver(rx, group, *chosen);
    std::cout << "Receiver configured\n";

    ma_device_config config    = ma_device_config_init(ma_device_type_playback);
    config.playback.pDeviceID  = &playbackInfos[outputDeviceIndex].id;
    config.playback.format     = ma_format_s16;
    config.playback.channels   = 0;     // native
    config.sampleRate          = senderSampleRate;
    config.periodSizeInFrames  = periodSizeInFrames;
    config.periods             = kNumPeriods;
    config.performanceProfile  = ma_performance_profile_low_latency;
    config.dataCallback        = data_callback;

    ma_device device;
    if (ma_device_init(&context, &config, &device) != MA_SUCCESS) {
        std::cerr << "Failed to open the selected output device\n";
        ma_context_uninit(&context);
        return 2;
    }
    if (device.sampleRate != senderSampleRate) {
        std::cerr << "Output device runs at " << device.sampleRate << " Hz, but the sender uses "
                  << senderSampleRate << " Hz\n";
        ma_device_uninit(&device);
        ma_context_uninit(&context);
        return 2;
    }
    std::cout << "playback: " << device.playback.channels << " ch s16 @ "
              << device.sampleRate << " Hz\n";

    const auto origin  = Clock::now();
    const auto seconds = [origin](Clock::time_point tp) {
        return std::chrono::duration<double>(tp - origin).count();
    };

    std::array<std::uint8_t, 8192> buf;
    TimeFilter filter;
    bool configured = false;
    int discard = 10;

    std::uint32_t expectedSeq = 0;
    std::uint64_t discontinuities = 0;

    std::function<void()> arm = [&] {
        rx.async_receive(asio::buffer(buf),
            [&](const asio::error_code& ec, std::size_t n) {
                if (ec == asio::error::operation_aborted) {
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

                const std::array<std::uint8_t, 8192> payload = [&] {
                    std::array<std::uint8_t, 8192> out{};
                    std::memcpy(out.data(), buf.data() + 16, n - 16);
                    return out;
                }();

                arm();

                const std::uint32_t gap = seq - expectedSeq;   // unsigned, wrap-safe
                if (gap != 0) {
                    ++discontinuities;
                    filter.skip(gap);
                }
                expectedSeq = seq + 1;

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
    arm();
    std::thread worker([&]{ io.run(); });

    if (ma_device_start(&device) != MA_SUCCESS) {
        std::cerr << "Failed to start the playback device\n";
        ma_device_uninit(&device);
        ma_context_uninit(&context);
        asio::post(io, [&]{ rx.close(); });
        worker.join();
        return 2;
    }

    std::cin.get();

    ma_device_stop(&device);
    ma_device_uninit(&device);
    ma_context_uninit(&context);

    asio::post(io, [&]{ rx.close(); });         // close from inside the io thread
    worker.join();

    return 0;
}

void enumerateOutputDevices(ma_uint32 playbackDeviceCount, ma_device_info *playbackInfos) {
    for (ma_uint32 deviceIndex = 0; deviceIndex < playbackDeviceCount; deviceIndex += 1) {
        std::cout << deviceIndex << " - " << playbackInfos[deviceIndex].name
                  << (playbackInfos[deviceIndex].isDefault ? " (default)" : "") << "\n";
    }
}

void enumerateNetworkInterfaces() {
    const auto all = net::enumerate();
    std::cout << "interfaces:\n";
    for (const auto &i : all)
    {
        std::cout << "  " << i.name << "  " << i.address.to_string()
                  << "  idx=" << i.index;
        if (!i.description.empty())
            std::cout << "  (" << i.description << ")";
        std::cout << "\n";
    }
}
