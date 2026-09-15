#include "clock_sync.h"
#include "netint.h"

#include <iostream>
#include <CLI/CLI.hpp>

int main(int argc, char** argv) {
    CLI::App app{"Clock sync demo"};
    argv = app.ensure_utf8(argv);  // proper Unicode handling on Windows

    std::string interface;
    uint64_t nodeId = 0;
    clocksync::Role role{clocksync::Role::Master};

    app.add_option("-i,--interface", interface, "Network interface");
    app.add_option("-n,--nodeid", nodeId, "Node ID")
        ->check(CLI::PositiveNumber);
    std::map<std::string, clocksync::Role> map{{"master", clocksync::Role::Master}, {"slave", clocksync::Role::Slave}};
    app.add_option("-r,--role", role, "Role (master or slave)")
        ->required()
        ->transform(CLI::CheckedTransformer(map, CLI::ignore_case));
    CLI11_PARSE(app, argc, argv);  // handles --help, errors, exit codes

    const auto all = net::enumerate();
    std::cout << "interfaces:\n";
    for (const auto& i : all) {
        std::cout << "  " << i.name << "  " << i.address.to_string()
                  << "  idx=" << i.index;
        if (!i.description.empty()) std::cout << "  (" << i.description << ")";
        std::cout << "\n";
    }
    const auto chosen = argc > 1 ? net::find(interface) : net::selectDefault();
    if (!chosen) {
        std::cerr << (argc > 1 ? "no such interface\n"
                               : "ambiguous or none; name one explicitly\n");
        return 1;
    }
    std::cout << "using " << chosen->name << " " << chosen->address.to_string() << "\n";

    clocksync::Config config{
        .iface = *chosen,
        .group = asio::ip::udp::endpoint(asio::ip::make_address("239.255.0.2"), 12345),
        .nodeId = nodeId,
    };
    clocksync::ClockSync cs{config, role};
    cs.start();

    getchar();
}