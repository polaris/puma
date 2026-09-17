#include "clock_sync.h"
#include "byte_order.h"

#include <iostream>

#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

namespace clocksync {
namespace {

/// Raise the calling thread to the highest non-realtime priority the platform
/// offers, so timestamping is not delayed by scheduling jitter.
void prioritizeCurrentThread() {
#if defined(_WIN32)
    ::SetThreadPriority(::GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#elif defined(__APPLE__)
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
}

} // namespace

ClockSync::ClockSync(Config config, Role role)
: socket_{io_}
, timer_{io_}
, receiver_{socket_}
, config_{config}
, role_{role}
, deadline_{Clock::now()}
, sequence_{0}
, seq_{0}
, localRef_{0}
, masterRef_{0}
, skew_{0}
, unmatched_{0}
, noSync_{0}
, staleSync_{0} {
    net::configureBidirectional(socket_, config.group, config.iface, {.hops = 1, .loopback = config.loopback});
}

void ClockSync::start() {
    if (worker_.joinable()) {
        return;
    }
    deadline_ = Clock::now();
    sequence_ = 0;
    servo_.configure(config_.acquireBandwidth, config_.lockBandwidth);
    socket_.non_blocking(true);
    // Only reports that the socket option was accepted. Whether the stamps are
    // usable is decided by probation, and reported by reportStampMode().
    (void)receiver_.enableKernelTimestamps();
    armReceive();
    if (role_ == Role::Master) {
        armSyncTimer();
    } else {
        armDelayReqTimer();
    }
    worker_ = std::thread([this] () {
        prioritizeCurrentThread();
        io_.run();
    });
}


ClockSync::~ClockSync() {
    stop();
}

void ClockSync::stop() {
    asio::post(io_, [this] {
        socket_.close();
        timer_.cancel();
    });
    if (worker_.joinable()) {
        worker_.join();
    }
}

[[nodiscard]] std::optional<ClockMapping> ClockSync::mapping() const noexcept {
    for (int tries = 0; tries < 4; ++tries) {
        const std::size_t s0 = seq_.load(std::memory_order_acquire);
        if (s0 == 0) {
            return std::nullopt;
        }
        if (s0 & 1) {
            continue;
        }
        ClockMapping m {
            .localRef = localRef_.load(std::memory_order_relaxed),
            .masterRef = masterRef_.load(std::memory_order_relaxed),
            .skew = skew_.load(std::memory_order_relaxed)
        };
        std::atomic_thread_fence(std::memory_order_acquire);
        if (seq_.load(std::memory_order_relaxed) == s0) {
            return m;
        }
    }
    return std::nullopt;
}

void ClockSync::updateMapping(double localRef, double masterRef, double skew) {
    const std::size_t s0 = seq_.load(std::memory_order_relaxed);
    seq_.store(s0 + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    localRef_.store(localRef, std::memory_order_relaxed);
    masterRef_.store(masterRef, std::memory_order_relaxed);
    skew_.store(skew, std::memory_order_relaxed);
    seq_.store(s0 + 2, std::memory_order_release);
}

[[nodiscard]] State ClockSync::state() const noexcept {
    return State{};
}

void ClockSync::reportStampMode() {
    // enableKernelTimestamps() only reports that the socket option was
    // accepted. Whether the stamps are usable is decided by probation, which
    // needs real datagrams, so the answer is only available here. Kernel mode
    // can later fall back to probation, so every change is reported.
    const auto mode = receiver_.mode();
    if (reportedMode_ == mode) return;
    if (!reportedMode_ && mode == StampMode::Probation) return;
    reportedMode_ = mode;

    const auto lagUs = std::chrono::duration_cast<std::chrono::microseconds>(
                           receiver_.kernelLag()).count();
    std::cerr << "timestamps: "
              << (mode == StampMode::Kernel      ? "kernel"
                : mode == StampMode::Probation ? "probation"
                                               : "userspace")
              << "  lag=" << lagUs << " us"
              << "  rate=" << receiver_.kernelRatePpm() << " ppm";
    if (const char* why = receiver_.rejectReason(); why && *why) {
        std::cerr << "  (" << why << ")";
    }
    std::cerr << "\n";
}

void ClockSync::publishStats() {
    statPathDelay_.store(toSeconds(servo_.pathDelay()), std::memory_order_relaxed);
    statGate_.store(toSeconds(servo_.gateThreshold()), std::memory_order_relaxed);
    statKernelLag_.store(toSeconds(receiver_.kernelLag()), std::memory_order_relaxed);
    statRejected_.store(servo_.rejected(), std::memory_order_relaxed);
    statTooSoon_.store(servo_.tooSoon(), std::memory_order_relaxed);
    statKernelStamps_.store(receiver_.kernelTimestamps(), std::memory_order_relaxed);
}

[[nodiscard]] Stats ClockSync::stats() const noexcept {
    Stats s;
    if (const auto m = mapping()) {
        s.offset = m->masterRef - m->localRef;
    }
    s.pathDelay       = statPathDelay_.load(std::memory_order_relaxed);
    s.gateThreshold   = statGate_.load(std::memory_order_relaxed);
    s.kernelLag       = statKernelLag_.load(std::memory_order_relaxed);
    s.kernelTimestamps= statKernelStamps_.load(std::memory_order_relaxed);
    s.rejected        = statRejected_.load(std::memory_order_relaxed);
    s.tooSoon         = statTooSoon_.load(std::memory_order_relaxed);
    s.unmatched       = unmatched_.load(std::memory_order_relaxed);
    s.noSync          = noSync_.load(std::memory_order_relaxed);
    s.staleSync       = staleSync_.load(std::memory_order_relaxed);

    // Cheap quality proxy until the Kalman filter supplies a covariance:
    // 1 when locked, decaying as the mapping ages.
    s.quality = (state().value == State::Locked) ? 1.0
              : (state().value == State::Acquiring) ? 0.5 : 0.0;
    return s;
}

void ClockSync::sendMessage(const SyncMessage& msg) {
    std::uint8_t buf[kMessageBytes];
    encode(msg, buf);
    asio::error_code ec;
    socket_.send_to(asio::buffer(buf, kMessageBytes), config_.group, 0, ec);
    if (ec) { /* TODO: count it, don't throw */ }
}

void ClockSync::armReceive() {
    socket_.async_wait(asio::ip::udp::socket::wait_read,
        [this](const asio::error_code& ec) {
            if (ec == asio::error::operation_aborted) return;
            if (ec) { armReceive(); return; }

            asio::error_code rec;
            const auto r = receiver_.receive(std::span{buffer_}, rec);
            if (!rec && r.bytes > 0) {
                if (!r.kernelStamp) userStamps_.fetch_add(1, std::memory_order_relaxed);
                remote_ = r.from;
                reportStampMode();          // one-shot, once probation settles
                handleReceive(r.bytes, r.stamp);
            }
            armReceive();
        });
}

void ClockSync::handleReceive(std::size_t n, Clock::time_point t) {
    const auto msg = decode(buffer_.data(), n);
    if (msg == std::nullopt) return;
    if (msg->domain != config_.domain) return;
    if (msg->nodeId == config_.nodeId) return;
    if (role_ == Role::Master) {
        if (msg->type == MsgType::DelayReq) {
            SyncMessage out{
                .type = MsgType::DelayResp,
                .domain = config_.domain,
                .nodeId = config_.nodeId,
                .targetId = msg->nodeId,
                .seq = sequence_++,
                .refSeq = msg->seq,
                .t = toNanos(t),
            };
            sendMessage(out);
        }
    } else {
        if (msg->type == MsgType::DelayResp) {
            if (msg->targetId != config_.nodeId) {
                return;
            }

            if (!pending_.valid || msg->refSeq != pending_.seq) {
                unmatched_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            pending_.valid = false;     

            if (!lastSync_.valid) {
                noSync_.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            const auto age = sinceEpoch(t) - lastSync_.t2;
            if (age > 2 * std::chrono::duration_cast<std::chrono::nanoseconds>(config_.syncInterval)) {
                staleSync_.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            const auto t4 = std::chrono::nanoseconds{static_cast<std::int64_t>(msg->t)};
            const auto a  = lastSync_.t2 - lastSync_.t1;
            const auto b  = t4 - pending_.t3;

            const auto offset = (a - b) / 2;
            const auto delay  = (a + b) / 2;

            const auto L = (pending_.t3 + lastSync_.t2) / 2;
            const auto M = L - offset;
            servo_.addSample(toSeconds(L), toSeconds(M), delay);
            publishStats();

            const auto mapping = servo_.mapping();

            if (mapping) {
                updateMapping(mapping->localRef, mapping->masterRef, mapping->skew);
                std::cout << toSeconds(offset) * 1e6 << " "                              // theta_raw, us
                          << toSeconds(delay)  * 1e6 << " "                              // delay, us
                          << -(mapping->masterRef - mapping->localRef) * 1e6 << " "      // theta from mapping
                          << mapping->skew * 1e6 << " "                                  // ppm
                          << servo_.rejected() << " "                                   // increments on reject
                          << servo_.tooSoon() << "\n";
            }

        } else if (msg->type == MsgType::Sync) {
            lastSync_ = {
                .t1 = std::chrono::nanoseconds{static_cast<std::int64_t>(msg->t)},
                .t2 = sinceEpoch(t),
                .valid = true
            };
        }
    }
}

void ClockSync::armSyncTimer() {
    deadline_ += config_.syncInterval;
    timer_.expires_at(deadline_);
    timer_.async_wait([this](const asio::error_code& ec) {
        if (ec == asio::error::operation_aborted) return;
        sendSync();
        armSyncTimer();
    });
}

void ClockSync::sendSync() {
    const auto t = Clock::now();
    SyncMessage msg{
        .type = MsgType::Sync,
        .domain = config_.domain,
        .nodeId = config_.nodeId,
        .seq = sequence_++,
        .t = toNanos(t),
    };
    sendMessage(msg);
}

void ClockSync::armDelayReqTimer() {
    deadline_ += config_.delayReqInterval;
    timer_.expires_at(deadline_);
    timer_.async_wait([this](const asio::error_code& ec) {
        if (ec == asio::error::operation_aborted) return;
        sendDelayReq();
        armDelayReqTimer();
    });
}

void ClockSync::sendDelayReq() {
    const auto t = Clock::now();
    SyncMessage msg{
        .type = MsgType::DelayReq,
        .domain = config_.domain,
        .nodeId = config_.nodeId,
        .seq = sequence_++,
        .t = toNanos(t),
    };
    pending_ = {
        .t3 = sinceEpoch(t),
        .seq = msg.seq,
        .valid = true
    };
    sendMessage(msg);
}

}