#include "timestamped_socket.h"

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

/// Space for the ancillary data. Generous: the largest payload we look for is
/// a timespec, and over-allocating costs nothing.
constexpr std::size_t kControlBytes = 256;

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
    enabled_ = false;

#if defined(__APPLE__)
    int on = 1;
    if (::setsockopt(socket_.native_handle(), SOL_SOCKET, SO_TIMESTAMP_MONOTONIC,
                     &on, sizeof(on)) == 0) {
        enabled_ = true;
    }

#elif defined(__linux__)
    int on = 1;
    if (::setsockopt(socket_.native_handle(), SOL_SOCKET, SO_TIMESTAMPNS,
                     &on, sizeof(on)) == 0) {
        refreshClockOffset();
        enabled_ = true;
    }

#elif defined(_WIN32)
    // The extension function pointer must be looked up per socket.
    GUID guid = WSAID_WSARECVMSG;
    LPFN_WSARECVMSG fn = nullptr;
    DWORD returned = 0;
    if (::WSAIoctl(socket_.native_handle(), SIO_GET_EXTENSION_FUNCTION_POINTER,
                   &guid, sizeof(guid), &fn, sizeof(fn),
                   &returned, nullptr, nullptr) == SOCKET_ERROR) {
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
        return false;
    }
    enabled_ = true;
#endif

    return enabled_;
}

// ---------------------------------------------------------------------------

#if defined(__linux__)
void TimestampedReceiver::refreshClockOffset() {
    timespec r{}, m{};
    ::clock_gettime(CLOCK_REALTIME, &r);
    ::clock_gettime(CLOCK_MONOTONIC, &m);
    const auto realtime  = std::chrono::seconds{r.tv_sec} + std::chrono::nanoseconds{r.tv_nsec};
    const auto monotonic = std::chrono::seconds{m.tv_sec} + std::chrono::nanoseconds{m.tv_nsec};
    realtimeToMonotonic_ =
        std::chrono::duration_cast<std::chrono::nanoseconds>(monotonic - realtime);
    offsetSampledAt_ = Clock::now();
}
#endif

std::chrono::steady_clock::time_point
TimestampedReceiver::convert(std::uint64_t raw) {
#if defined(__APPLE__)
    // raw is mach_absolute_time ticks; steady_clock uses the same timebase.
    const std::uint64_t ns = raw * machNumer_ / machDenom_;
    return Clock::time_point{std::chrono::nanoseconds{static_cast<std::int64_t>(ns)}};

#elif defined(__linux__)
    // raw is CLOCK_REALTIME nanoseconds; shift into CLOCK_MONOTONIC.
    if (Clock::now() - offsetSampledAt_ > std::chrono::seconds{2}) refreshClockOffset();
    const auto realtime = std::chrono::nanoseconds{static_cast<std::int64_t>(raw)};
    return Clock::time_point{realtime + realtimeToMonotonic_};

#elif defined(_WIN32)
    // raw is QPC ticks; MSVC's steady_clock is QPC-based.
    // Split into whole seconds and remainder to avoid overflowing 64 bits, as
    // MSVC's own steady_clock::now() does.
    if (qpcFrequency_ == 0) return Clock::now();
    const auto ticks = static_cast<std::int64_t>(raw);
    const auto ns = (ticks / qpcFrequency_) * 1'000'000'000LL +
                    (ticks % qpcFrequency_) * 1'000'000'000LL / qpcFrequency_;
    return Clock::time_point{std::chrono::nanoseconds{ns}};

#else
    (void)raw;
    return Clock::now();
#endif
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
    out.stamp = Clock::now();

    if (n < 0) {
        ec = asio::error_code(errno, asio::error::get_system_category());
        return out;
    }
    ec.clear();
    out.bytes = static_cast<std::size_t>(n);
    out.from  = toEndpoint(src);

    for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level != SOL_SOCKET) continue;

#if defined(__APPLE__)
        if (c->cmsg_type == SCM_TIMESTAMP_MONOTONIC) {
            std::uint64_t ticks = 0;
            std::memcpy(&ticks, CMSG_DATA(c), sizeof(ticks));
            out.stamp = convert(ticks);
            out.kernelStamp = true;
            break;
        }
#elif defined(__linux__)
        if (c->cmsg_type == SCM_TIMESTAMPNS) {
            timespec ts{};
            std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            const auto ns = static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ull +
                            static_cast<std::uint64_t>(ts.tv_nsec);
            out.stamp = convert(ns);
            out.kernelStamp = true;
            break;
        }
#endif
    }
    return out;
}

#else  // _WIN32

RxResult TimestampedReceiver::receive(std::span<std::uint8_t> buffer,
                                      asio::error_code& ec) {
    RxResult out;

    if (recvMsgFn_ == nullptr) {
        // Kernel timestamping was never enabled; fall back to a plain read.
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

    out.stamp = Clock::now();

    if (rc == SOCKET_ERROR) {
        ec = asio::error_code(::WSAGetLastError(), asio::error::get_system_category());
        return out;
    }
    ec.clear();
    out.bytes = received;
    out.from  = toEndpoint(src);

    for (WSACMSGHDR* c = WSA_CMSG_FIRSTHDR(&msg); c != nullptr;
         c = WSA_CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMP) {
            UINT64 ticks = 0;
            std::memcpy(&ticks, WSA_CMSG_DATA(c), sizeof(ticks));
            out.stamp = convert(ticks);
            out.kernelStamp = true;
            break;
        }
    }
    return out;
}

#endif

}  // namespace clocksync