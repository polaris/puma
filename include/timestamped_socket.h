#ifndef TIMESTAMPED_SOCKET_H
#define TIMESTAMPED_SOCKET_H

#include <asio.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace clocksync {

/// The result of one datagram read.
struct RxResult {
    std::size_t bytes = 0;
    asio::ip::udp::endpoint from;
    std::chrono::steady_clock::time_point stamp{};
    bool kernelStamp = false;   ///< false => stamp was taken in userspace
};

/// Reads datagrams with the best receive timestamp the platform offers.
///
/// asio is still used for readiness (async_wait) and for socket lifetime; the
/// read itself goes through recvmsg / WSARecvMsg so the ancillary data carrying
/// the kernel timestamp is reachable.
///
/// If kernel timestamping is unavailable the class still works: `stamp` is then
/// taken in userspace immediately after the read returns and `kernelStamp` is
/// false. Report that flag: a pair of nodes where only one side has kernel
/// timestamps has a structurally asymmetric path, which biases the offset.
class TimestampedReceiver {
public:
    explicit TimestampedReceiver(asio::ip::udp::socket& socket);

    /// Ask the kernel to stamp incoming datagrams.
    /// Returns false if the platform or the NIC driver does not support it;
    /// that is not an error, it selects the userspace fallback.
    bool enableKernelTimestamps();

    [[nodiscard]] bool kernelTimestamps() const noexcept { return enabled_; }

    /// One non-blocking read. On success ec is cleared and bytes > 0.
    /// ec == asio::error::would_block means the socket was not actually
    /// readable, which async_wait can report; the caller should just re-arm.
    RxResult receive(std::span<std::uint8_t> buffer, asio::error_code& ec);

private:
    asio::ip::udp::socket& socket_;
    bool enabled_ = false;

    /// Convert a platform-native raw timestamp into the steady_clock domain.
    std::chrono::steady_clock::time_point convert(std::uint64_t raw);

#if defined(__linux__)
    // Linux software timestamps are CLOCK_REALTIME; steady_clock is
    // CLOCK_MONOTONIC. Keep a periodically refreshed offset between them.
    void refreshClockOffset();
    std::chrono::nanoseconds realtimeToMonotonic_{0};
    std::chrono::steady_clock::time_point offsetSampledAt_{};
#endif
#if defined(__APPLE__)
    std::uint64_t machNumer_ = 1, machDenom_ = 1;
#endif
#if defined(_WIN32)
    std::int64_t qpcFrequency_ = 0;
    void* recvMsgFn_ = nullptr;   // LPFN_WSARECVMSG, kept type-erased
#endif
};

}  // namespace clocksync

#endif  // TIMESTAMPED_SOCKET_H