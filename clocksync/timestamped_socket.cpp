#include "timestamped_socket.h"

#include <algorithm>
#include <vector>
#include <cstring>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mswsock.h>
#include <mstcpip.h>
#include <windows.h>
#else
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <cerrno>
#include <ctime>
#endif

#if defined(__APPLE__)
#include <mach/mach_time.h>
#endif

namespace clocksync {
namespace {

using Clock = std::chrono::steady_clock;

/// Build an asio endpoint from a raw sockaddr (IPv4 only, as elsewhere).
asio::ip::udp::endpoint toEndpoint(const sockaddr_storage& ss) {
    if (ss.ss_family != AF_INET) return {};
    const auto* sin = reinterpret_cast<const sockaddr_in*>(&ss);
    asio::ip::address_v4::bytes_type bytes{};
    std::memcpy(bytes.data(), &sin->sin_addr, bytes.size());
    return {asio::ip::address_v4(bytes), ntohs(sin->sin_port)};
}

/// Space for the ancillary data. Generous; over-allocating costs nothing.
constexpr std::size_t kControlBytes = 256;

/// Probation thresholds. The lag is the receive-path latency, so it must be
/// positive and of a plausible magnitude, and it must not trend grossly. The
/// drift test only spans 16 datagrams and medians move with load, so it is
/// kept loose; small rate errors are the job of trackRate().
constexpr auto kMinLag   = std::chrono::microseconds{-200};
constexpr auto kMaxLag   = std::chrono::milliseconds{50};
constexpr auto kMaxDrift = std::chrono::milliseconds{5};

/// Current value of the clock the kernel stamps datagrams with, in its raw unit.
std::uint64_t readRawClock() {
#if defined(__APPLE__)
    return mach_absolute_time();
#elif defined(__linux__)
    timespec r{};
    ::clock_gettime(CLOCK_REALTIME, &r);
    return static_cast<std::uint64_t>(r.tv_sec) * 1'000'000'000ull +
           static_cast<std::uint64_t>(r.tv_nsec);
#elif defined(_WIN32)
    LARGE_INTEGER qpc{};
    QueryPerformanceCounter(&qpc);
    return static_cast<std::uint64_t>(qpc.QuadPart);
#else
    return 0;
#endif
}

}  // namespace

// ---------------------------------------------------------------------------

TimestampedReceiver::TimestampedReceiver(asio::ip::udp::socket& socket)
    : socket_{socket} {
#if defined(__APPLE__)
    mach_timebase_info_data_t tb{};
    if (mach_timebase_info(&tb) == KERN_SUCCESS && tb.denom != 0) {
        machNumer_ = tb.numer;
        machDenom_ = tb.denom;
    }
#elif defined(_WIN32)
    LARGE_INTEGER f{};
    if (QueryPerformanceFrequency(&f)) qpcFrequency_ = f.QuadPart;
#endif
}

// ---------------------------------------------------------------------------

bool TimestampedReceiver::enableKernelTimestamps() {
    mode_ = StampMode::Userspace;
    rejectReason_ = "";
    probeCount_ = 0;
    unstamped_ = 0;
    strikes_ = 0;
    window_ = {};
    rateRef_.reset();
    lastFloor_.reset();
    ratePpm_ = 0.0;

#if defined(__APPLE__)
    int on = 1;
    if (::setsockopt(socket_.native_handle(), SOL_SOCKET, SO_TIMESTAMP_MONOTONIC,
                     &on, sizeof(on)) != 0) {
        rejectReason_ = "setsockopt(SO_TIMESTAMP_MONOTONIC) failed";
        return false;
    }

#elif defined(__linux__)
    int on = 1;
    if (::setsockopt(socket_.native_handle(), SOL_SOCKET, SO_TIMESTAMPNS,
                     &on, sizeof(on)) != 0) {
        rejectReason_ = "setsockopt(SO_TIMESTAMPNS) failed";
        return false;
    }

#elif defined(_WIN32)
    // The extension function pointer must be looked up per socket.
    GUID guid = WSAID_WSARECVMSG;
    LPFN_WSARECVMSG fn = nullptr;
    DWORD returned = 0;
    if (::WSAIoctl(socket_.native_handle(), SIO_GET_EXTENSION_FUNCTION_POINTER,
                   &guid, sizeof(guid), &fn, sizeof(fn),
                   &returned, nullptr, nullptr) == SOCKET_ERROR) {
        rejectReason_ = "WSARecvMsg lookup failed";
        return false;
    }
    recvMsgFn_ = reinterpret_cast<void*>(fn);

    TIMESTAMPING_CONFIG cfg{};
    cfg.Flags = TIMESTAMPING_FLAG_RX;
    cfg.TxTimestampsBuffered = 0;
    DWORD bytes = 0;
    if (::WSAIoctl(socket_.native_handle(), SIO_TIMESTAMPING,
                   &cfg, sizeof(cfg), nullptr, 0,
                   &bytes, nullptr, nullptr) == SOCKET_ERROR) {
        // WSAEOPNOTSUPP (10045): the NIC driver does not expose timestamping.
        rejectReason_ = "SIO_TIMESTAMPING not supported by this NIC";
        return false;
    }

#else
    rejectReason_ = "no kernel timestamping on this platform";
    return false;
#endif

    refreshClockOffset();
    mode_ = StampMode::Probation;
    return true;
}

// ---------------------------------------------------------------------------

std::chrono::nanoseconds TimestampedReceiver::rawToNanos(std::uint64_t raw) const {
#if defined(__APPLE__)
    // mach_absolute_time ticks
    return std::chrono::nanoseconds{
        static_cast<std::int64_t>(raw * machNumer_ / machDenom_)};

#elif defined(__linux__)
    // already nanoseconds, in CLOCK_REALTIME
    return std::chrono::nanoseconds{static_cast<std::int64_t>(raw)};

#elif defined(_WIN32)
    // QPC ticks. Split into whole seconds and remainder so the intermediate
    // product cannot overflow 64 bits.
    if (qpcFrequency_ == 0) return std::chrono::nanoseconds{0};
    const auto ticks = static_cast<std::int64_t>(raw);
    const auto ns = (ticks / qpcFrequency_) * 1'000'000'000LL +
                    (ticks % qpcFrequency_) * 1'000'000'000LL / qpcFrequency_;
    return std::chrono::nanoseconds{ns};

#else
    (void)raw;
    return std::chrono::nanoseconds{0};
#endif
}

void TimestampedReceiver::refreshClockOffset() {
    // Bracket the raw read between two steady_clock reads and pair it with the
    // midpoint. Whatever the raw clock's epoch is, this puts converted stamps
    // into the steady_clock domain. A preemption inside the bracket would skew
    // the offset for the next 2 s, so keep the tightest of a few attempts.
    auto best = std::chrono::nanoseconds::max();
    for (int i = 0; i < 3; ++i) {
        const auto s0  = Clock::now();
        const auto raw = readRawClock();
        const auto s1  = Clock::now();
        if (s1 - s0 < best) {
            best = s1 - s0;
            clockOffset_ = (s0.time_since_epoch() + (s1 - s0) / 2) - rawToNanos(raw);
            offsetSampledAt_ = s1;
        }
    }
}

std::chrono::steady_clock::time_point
TimestampedReceiver::convert(std::uint64_t raw, Clock::time_point now) {
    if (now - offsetSampledAt_ > std::chrono::seconds{2}) {
        refreshClockOffset();
    }
    return Clock::time_point{rawToNanos(raw) + clockOffset_};
}

// ---------------------------------------------------------------------------

void TimestampedReceiver::addProbe(std::chrono::nanoseconds lag) {
    probes_[probeCount_++] = lag;
    if (probeCount_ == kProbeCount) decideMode();
}

void TimestampedReceiver::decideMode() {
    auto median = [](auto first, auto last) {
        std::vector<std::chrono::nanoseconds> v(first, last);
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };

    const auto half = probes_.begin() + kProbeCount / 2;
    const auto all    = median(probes_.begin(), probes_.begin() + kProbeCount);
    const auto first  = median(probes_.begin(), half);
    const auto second = median(half, probes_.begin() + kProbeCount);
    const auto drift  = second - first;

    kernelLag_ = all;

    if (all < kMinLag) {
        rejectReason_ = "kernel stamp is ahead of the userspace stamp "
                        "(clock domains differ)";
        mode_ = StampMode::Userspace;
        return;
    }
    if (all > kMaxLag) {
        rejectReason_ = "kernel stamp lags implausibly far behind "
                        "(clock domains differ)";
        mode_ = StampMode::Userspace;
        return;
    }
    if (drift > kMaxDrift || drift < -kMaxDrift) {
        rejectReason_ = "kernel clock runs at a different rate from steady_clock";
        mode_ = StampMode::Userspace;
        return;
    }

    rejectReason_ = "";
    mode_ = StampMode::Kernel;
}

void TimestampedReceiver::strike(const char* reason) {
    if (++strikes_ < kMaxStrikes) return;
    // Back to probation rather than straight to userspace: a transient problem
    // recovers, a genuinely broken clock domain is rejected by decideMode().
    rejectReason_ = reason;
    mode_ = StampMode::Probation;
    probeCount_ = 0;
    unstamped_ = 0;
    strikes_ = 0;
}

void TimestampedReceiver::trackRate(Clock::time_point userStamp, std::uint64_t raw) {
    // No offset applied: d is trueOffset + latency + rate * t, so the periodic
    // offset refresh cannot hide a rate error, and the latency floor (the
    // per-window minimum) is far steadier than a median.
    const auto d = userStamp.time_since_epoch() - rawToNanos(raw);

    if (window_.min == std::chrono::nanoseconds::max()) window_.at = userStamp;
    window_.min = std::min(window_.min, d);
    if (userStamp - window_.at < kRateWindow) return;

    const Floor floor = window_;
    window_ = {};

    const bool jumped = lastFloor_ &&
        std::chrono::abs(floor.min - lastFloor_->min) > kRateJump;
    lastFloor_ = floor;
    if (!rateRef_ || jumped) {
        rateRef_ = floor;   // (re)start the baseline
        return;
    }

    const auto span = floor.at - rateRef_->at;
    if (span < kRateBaseline) return;

    ratePpm_ = 1e6 * static_cast<double>((floor.min - rateRef_->min).count()) /
               static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(span).count());
    rateRef_ = floor;   // slide the baseline

    if (std::abs(ratePpm_) > kMaxRatePpm) {
        // A rate error does not go away, so re-probing would only flap.
        rejectReason_ = "kernel clock runs at a different rate from steady_clock";
        mode_ = StampMode::Userspace;
    }
}

// ---------------------------------------------------------------------------

void TimestampedReceiver::applyStamp(RxResult& out,
                                     std::optional<std::uint64_t> raw) {
    const auto userStamp = out.stamp;

    if (mode_ == StampMode::Userspace) return;

    if (raw) {
        trackRate(userStamp, *raw);
        if (mode_ == StampMode::Userspace) return;
    }

    if (mode_ == StampMode::Probation) {
        if (!raw) {
            // The option was accepted but stamps never arrive (seen with some
            // Windows NIC drivers). Don't sit in probation forever.
            if (++unstamped_ >= kMaxUnstamped) {
                rejectReason_ = "option accepted but no kernel timestamps delivered";
                mode_ = StampMode::Userspace;
            }
            return;
        }
        // Validate before trusting: keep reporting the userspace stamp.
        addProbe(userStamp - convert(*raw, userStamp));
        return;
    }

    // Kernel mode: every stamp is checked against the userspace stamp.
    if (!raw) {
        strike("kernel timestamps stopped arriving");
        return;
    }

    auto kernelStamp = convert(*raw, userStamp);
    auto lag = userStamp - kernelStamp;

    if (lag < kMinLag || lag > kMaxLag) {
        // Most likely a clock step (Linux CLOCK_REALTIME) or a macOS wake that
        // moved the offset. Re-measure now instead of up to 2 s later.
        refreshClockOffset();
        kernelStamp = convert(*raw, userStamp);
        lag = userStamp - kernelStamp;
    }

    if (lag < kMinLag) {
        // Stamped after we read the packet: impossible, so the stamp is wrong.
        // Never report it; out.stamp stays the userspace stamp.
        strike("kernel stamp is ahead of the userspace stamp");
        return;
    }

    if (lag > kMaxLag) {
        // Ambiguous: a real stall of this thread looks the same, and then the
        // kernel stamp is exactly the right one. Use it, but count it.
        strike("kernel stamp persistently lags implausibly far behind");
    } else {
        strikes_ = 0;
    }

    out.stamp = kernelStamp;
    out.kernelStamp = true;
}

// ---------------------------------------------------------------------------

#if !defined(_WIN32)

RxResult TimestampedReceiver::receive(std::span<std::uint8_t> buffer,
                                      asio::error_code& ec) {
    RxResult out;

    sockaddr_storage src{};
    iovec iov{};
    iov.iov_base = buffer.data();
    iov.iov_len  = buffer.size();

    alignas(alignof(cmsghdr)) char control[kControlBytes];

    msghdr msg{};
    msg.msg_name       = &src;
    msg.msg_namelen    = sizeof(src);
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = control;
    msg.msg_controllen = sizeof(control);   // must be reset on every call

    const ssize_t n = ::recvmsg(socket_.native_handle(), &msg, 0);

    // Userspace fallback timestamp, taken as early as possible.
    const auto userStamp = Clock::now();
    out.stamp = userStamp;

    if (n < 0) {
        ec = asio::error_code(errno, asio::error::get_system_category());
        return out;
    }
    if (msg.msg_flags & MSG_TRUNC) {
        // Datagram larger than the buffer: never hand out a partial payload.
        ec = asio::error::message_size;
        return out;
    }
    ec.clear();
    out.bytes = static_cast<std::size_t>(n);
    out.from  = toEndpoint(src);

    if (mode_ == StampMode::Userspace) return out;
    if (msg.msg_flags & MSG_CTRUNC) {
        // The timestamp may have been cut off. That is a configuration error
        // (more ancillary options than kControlBytes holds), not a clock problem.
        rejectReason_ = "control buffer too small for timestamp";
        mode_ = StampMode::Userspace;
        return out;
    }

    std::optional<std::uint64_t> raw;
    for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level != SOL_SOCKET) continue;
#if defined(__APPLE__)
        if (c->cmsg_type == SCM_TIMESTAMP_MONOTONIC) {
            std::uint64_t v = 0;
            std::memcpy(&v, CMSG_DATA(c), sizeof(v));
            raw = v;
            break;
        }
#elif defined(__linux__)
        if (c->cmsg_type == SCM_TIMESTAMPNS) {
            timespec ts{};
            std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            raw = static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ull +
                  static_cast<std::uint64_t>(ts.tv_nsec);
            break;
        }
#endif
    }
    applyStamp(out, raw);
    return out;
}

#else  // _WIN32

RxResult TimestampedReceiver::receive(std::span<std::uint8_t> buffer,
                                      asio::error_code& ec) {
    RxResult out;

    if (recvMsgFn_ == nullptr) {
        out.bytes = socket_.receive_from(asio::buffer(buffer.data(), buffer.size()),
                                         out.from, 0, ec);
        out.stamp = Clock::now();
        return out;
    }

    auto WSARecvMsgFn = reinterpret_cast<LPFN_WSARECVMSG>(recvMsgFn_);

    sockaddr_storage src{};
    WSABUF buf{};
    buf.buf = reinterpret_cast<CHAR*>(buffer.data());
    buf.len = static_cast<ULONG>(buffer.size());

    alignas(8) char control[kControlBytes];

    WSAMSG msg{};
    msg.name          = reinterpret_cast<LPSOCKADDR>(&src);
    msg.namelen       = sizeof(src);
    msg.lpBuffers     = &buf;
    msg.dwBufferCount = 1;
    msg.Control.buf   = control;
    msg.Control.len   = sizeof(control);
    msg.dwFlags       = 0;

    DWORD received = 0;
    const int rc = WSARecvMsgFn(socket_.native_handle(), &msg, &received,
                                nullptr, nullptr);

    const auto userStamp = Clock::now();
    out.stamp = userStamp;

    if (rc == SOCKET_ERROR) {
        // Includes WSAEMSGSIZE: a datagram larger than the buffer never succeeds.
        ec = asio::error_code(::WSAGetLastError(), asio::error::get_system_category());
        return out;
    }
    ec.clear();
    out.bytes = received;
    out.from  = toEndpoint(src);

    if (mode_ == StampMode::Userspace) return out;
    if (msg.dwFlags & MSG_CTRUNC) {
        rejectReason_ = "control buffer too small for timestamp";
        mode_ = StampMode::Userspace;
        return out;
    }

    std::optional<std::uint64_t> raw;
    for (WSACMSGHDR* c = WSA_CMSG_FIRSTHDR(&msg); c != nullptr;
         c = WSA_CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMP) {
            UINT64 v = 0;
            std::memcpy(&v, WSA_CMSG_DATA(c), sizeof(v));
            raw = v;
            break;
        }
    }
    applyStamp(out, raw);
    return out;
}

#endif

}  // namespace clocksync