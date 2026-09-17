#ifndef TIMESTAMPED_SOCKET_H
#define TIMESTAMPED_SOCKET_H

#include <asio.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
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
///    frequency. A probation phase checks the stamps against live traffic
///    before they are trusted, every stamp is sanity-checked afterwards, and a
///    long-baseline rate estimate catches clocks that run slightly fast or slow.
///    Misbehaving stamps fall back to userspace.
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

    /// Why probation rejected the stamps, or why Kernel mode was abandoned for
    /// re-validation; "" if neither happened since the stamps were last accepted.
    [[nodiscard]] const char* rejectReason() const noexcept { return rejectReason_; }

    /// Rate of the kernel stamp clock relative to steady_clock, in ppm, as
    /// currently corrected for in convert(). 0 until two consecutive
    /// kRateBaseline estimates have agreed within kRateAgreePpm.
    [[nodiscard]] double kernelRatePpm() const noexcept { return ratePpm_; }

    /// One non-blocking read. On success ec is cleared and bytes > 0. Any error
    /// (including would-block, which async_wait can produce) means "no
    /// datagram"; the caller should simply re-arm.
    RxResult receive(std::span<std::uint8_t> buffer, asio::error_code& ec);

private:
    static constexpr std::size_t kProbeCount   = 16;
    static constexpr std::size_t kMaxUnstamped = 64;  ///< probation reads without a stamp before giving up
    static constexpr std::size_t kMaxStrikes   = 8;   ///< consecutive bad stamps before re-validating

    // Rate check: floor of (userspace - raw kernel) per window, compared across
    // a long baseline.
    static constexpr std::chrono::seconds      kRateWindow{1};
    static constexpr std::chrono::seconds      kRateBaseline{30};
    // Sanity bound only: convert() extrapolates an agreed rate away, so what is
    // left to catch here is a clock that is not merely off-frequency but wrong,
    // well outside crystal tolerance.
    static constexpr double kMaxRatePpm = 100.0;
    // A clock within kMaxRatePpm moves the floor by at most 100 µs per window.
    // Anything larger is a step (sleep, clock step, or a shift in the latency
    // floor itself, e.g. CPU idle states or Wi-Fi power save), not a rate, and
    // restarts the baseline instead of being read as a slope.
    static constexpr std::chrono::microseconds kRateJump{150};
    static_assert(kMaxRatePpm * 1e-6 *
                      static_cast<double>(std::chrono::microseconds{kRateWindow}.count()) <
                  static_cast<double>(kRateJump.count()),
                  "kRateJump must exceed the floor movement a kMaxRatePpm clock can cause");
    // A floor shift below kRateJump still lands in one baseline as a false
    // slope, but not in the next. Estimates are acted on only when two
    // consecutive ones agree, which a real (stable) rate does and a one-off
    // shift does not.
    static constexpr double kRateAgreePpm = 1.0;

    /// Raw platform timestamp -> nanoseconds in the raw clock's own domain.
    [[nodiscard]] std::chrono::nanoseconds rawToNanos(std::uint64_t raw) const;

    /// Measure (steady_clock - raw clock) so converted stamps land in the
    /// steady_clock domain. Re-sampled periodically: on macOS this offset moves
    /// whenever the machine sleeps.
    void refreshClockOffset();

    /// `now` is the caller's userspace stamp; it only decides whether the
    /// offset is stale, so saving a clock read per datagram is worth the µs.
    [[nodiscard]] std::chrono::steady_clock::time_point
    convert(std::uint64_t raw, std::chrono::steady_clock::time_point now);

    /// Mode-dependent handling of one received datagram's stamp. `out.stamp`
    /// holds the userspace stamp on entry and is replaced only if the kernel
    /// stamp is trusted and passes the per-packet sanity check.
    void applyStamp(RxResult& out, std::optional<std::uint64_t> raw);

    void addProbe(std::chrono::nanoseconds lag);
    void decideMode();

    /// Count one suspect stamp while in Kernel mode; after kMaxStrikes in a
    /// row, drop back to Probation so the stamps are re-validated.
    void strike(const char* reason);

    /// Feed one stamped datagram to the long-baseline rate estimate; rejects
    /// the kernel stamps if the rate exceeds kMaxRatePpm.
    void trackRate(std::chrono::steady_clock::time_point userStamp, std::uint64_t raw);

    asio::ip::udp::socket& socket_;
    StampMode mode_ = StampMode::Userspace;
    const char* rejectReason_ = "";

    std::chrono::nanoseconds clockOffset_{0};
    std::chrono::steady_clock::time_point offsetSampledAt_{};

    std::array<std::chrono::nanoseconds, kProbeCount> probes_{};
    std::size_t probeCount_ = 0;
    std::size_t unstamped_ = 0;
    std::size_t strikes_ = 0;
    std::chrono::nanoseconds kernelLag_{0};

    struct Floor {
        std::chrono::steady_clock::time_point at{};
        std::chrono::nanoseconds min = std::chrono::nanoseconds::max();
    };
    Floor window_{};
    std::optional<Floor> rateRef_, lastFloor_;
    std::optional<double> lastEstimate_;   ///< previous baseline's estimate, ppm
    double ratePpm_ = 0.0;                 ///< agreed rate, applied in convert()

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