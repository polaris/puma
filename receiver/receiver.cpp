
#include <asio.hpp>
#include <chrono>
#include <iostream>
#include <thread>

#include "netint.h"
#include "time_filter.h"

using asio::ip::udp;
using Clock = std::chrono::steady_clock;

constexpr double kBandwidth = 0.05;

int main(int argc, char** argv) {
    const auto all = net::enumerate();
    std::cout << "interfaces:\n";
    for (const auto& i : all) {
        std::cout << "  " << i.name << "  " << i.address.to_string()
                  << "  idx=" << i.index;
        if (!i.description.empty()) std::cout << "  (" << i.description << ")";
        std::cout << "\n";
    }
    const auto chosen = argc > 1 ? net::find(argv[1]) : net::selectDefault();
    if (!chosen) {
        std::cerr << (argc > 1 ? "no such interface\n"
                               : "ambiguous or none; name one explicitly\n");
        return 1;
    }
    std::cout << "using " << chosen->name << " " << chosen->address.to_string() << "\n";

    const asio::ip::udp::endpoint group(asio::ip::make_address("239.255.0.1"), 12345);
    asio::io_context io;
    asio::ip::udp::socket rx(io);
    net::configureReceiver(rx, group, *chosen);
    std::cout << "sender and receiver configured\n";
    std::cout << "Receiver configured\n";

    const auto origin  = Clock::now();
    const auto seconds = [origin](Clock::time_point tp) {
        return std::chrono::duration<double>(tp - origin).count();
    };

    const double nominalRate = 44100.0;  // TODO: query the sender for this

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
                    filter.configure(kBandwidth, frames, nominalRate);
                    filter.reset(t);
                    configured = true;
                    std::fprintf(stderr, "filtering %u frames/callback at nominal %.0f Hz\n", frames, nominalRate);
                    return;
                }

                if (frames != filter.framesPerPeriod()) {
                    std::fprintf(stderr, "frame count changed %u -> %u, resyncing\n", filter.framesPerPeriod(), frames);
                    filter.configure(kBandwidth, frames, nominalRate);
                    filter.invalidate();
                }

                if (!filter.ready()) {
                    filter.reset(t);
                    return;
                }

                filter.update(t);

                std::printf("%llu %.9f %.9f %.12f %.4f\n", static_cast<unsigned long long>(filter.frame()), filter.time(), filter.error(), filter.period(), filter.rate());
            });
    };
    arm();
    std::thread worker([&]{ io.run(); });

    std::cin.get();

    asio::post(io, [&]{ rx.close(); });         // close from inside the io thread
    worker.join();

    return 0;
}