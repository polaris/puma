#include "netint.h"
#include <iostream>

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
    asio::ip::udp::socket tx(io), rx(io);
    net::configureSender(tx, group, *chosen, {.hops = 1, .loopback = true});
    net::configureReceiver(rx, group, *chosen);
    std::cout << "sender and receiver configured\n";
}
