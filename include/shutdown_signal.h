#ifndef SHUTDOWN_SIGNAL_H
#define SHUTDOWN_SIGNAL_H

#include <asio.hpp>

#include <csignal>

// Runs `control` on the calling thread until SIGINT or SIGTERM.
inline void runUntilShutdownSignal(asio::io_context& control) {
    asio::signal_set signals{control, SIGINT, SIGTERM};
    signals.async_wait([&](auto, int) {
        control.stop();
    });
    control.run();
}

#endif  // SHUTDOWN_SIGNAL_H
