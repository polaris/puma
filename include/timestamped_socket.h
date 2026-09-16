#ifndef TIMESTAMPED_SOCKET_H
#define TIMESTAMPED_SOCKET_H

#include <asio.hpp>

#include <array>
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

/// Which timestamp source is currently in use.
enum class StampMode {
    Userspace,   ///< kernel timestamps unavailable, or rejected by probation
    Probation,   ///< kernel stamps are being validated; userspace still in use
    Kernel,      ///< validated, kernel stamps in use
};

/// Reads datagrams with the best receive timestamp the platform offers.
///
/// asio provides readiness (async_wait) and socket lifetime; the read goes
/// through recvmsg / WSARecvMsg so the ancillary data carrying the kernel
/// timestamp is reachable.
///
/// Two things this class deliberately does NOT assume, both of which turned out
/// to be false in practice:
///
///  * that the kernel timestamp shares steady_clock's epoch. On macOS the
///    socket stamp is mach_absolute_time while libc++'s steady_clock is
///    CLOCK_MONOTONIC_RAW, and those differ by accumulated sleep time. On Linux
///    the stamp is CLOCK_REALTIME. The offset is measured, not assumed.
///
///  * that the kernel timestamp advances at the same rate as steady_clock. A
///    NIC providing a hardware timestamp reports its own clock at its own
///    frequency. A probation phase checks this against live traffic before the
///    stamps are trusted, and falls back to userspace if they misbehave.
class TimestampedReceiver {
public:
    explicit TimestampedReceiver(asio::ip::udp::socket& socket);

    /// Ask the kernel to stamp incoming datagrams. Returns false if the
    /// platform or the NIC driver does not support it. Returning true means the
    /// option was accepted, not that the stamps are usable: the first
    /// kProbeCount datagrams validate them while userspace stamps are still
    /// what gets reported.
    bool enableKernelTimestamps();

    [[nodiscard]] StampMode mode() const noexcept { return mode_; }
    [[nodiscard]] bool kernelTimestamps() const noexcept {
        return mode_ == StampMode::Kernel;
    }

    /// Median (userspace stamp - kernel stamp) measured during probation: the
    /// receive-path latency that kernel timestamping removes.
    [[nodiscard]] std::chrono::nanoseconds kernelLag() const noexcept { return kernelLag_; }

    /// Why probation rejected the stamps, or "" if it did not.
    [[nodiscard]] const char* rejectReason() const noexcept { return rejectReason_; }

    /// One non-blocking read. On success ec is cleared and bytes > 0. Any error
    /// (including would-block, which async_wait can produce) means "no
    /// datagram"; the caller should simply re-arm.
    RxResult receive(std::span<std::uint8_t> buffer, asio::error_code& ec);

private:
    static constexpr std::size_t kProbeCount = 16;

    /// Raw platform timestamp -> nanoseconds in the raw clock's own domain.
    [[nodiscard]] std::chrono::nanoseconds rawToNanos(std::uint64_t raw) const;

    /// Measure (steady_clock - raw clock) so converted stamps land in the
    /// steady_clock domain. Re-sampled periodically: on macOS this offset moves
    /// whenever the machine sleeps.
    void refreshClockOffset();

    [[nodiscard]] std::chrono::steady_clock::time_point convert(std::uint64_t raw);

    void addProbe(std::chrono::nanoseconds lag);
    void decideMode();

    asio::ip::udp::socket& socket_;
    StampMode mode_ = StampMode::Userspace;
    const char* rejectReason_ = "";

    std::chrono::nanoseconds clockOffset_{0};
    std::chrono::steady_clock::time_point offsetSampledAt_{};

    std::array<std::chrono::nanoseconds, kProbeCount> probes_{};
    std::size_t probeCount_ = 0;
    std::chrono::nanoseconds kernelLag_{0};

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