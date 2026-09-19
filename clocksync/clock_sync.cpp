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
    encode({ .type = MsgType::Sync,      .domain = config_.domain, .nodeId = config_.nodeId, }, syncMessageBuffer_);
    encode({ .type = MsgType::DelayReq,  .domain = config_.domain, .nodeId = config_.nodeId, }, delayReqMessageBuffer_);
    encode({ .type = MsgType::DelayResp, .domain = config_.domain, .nodeId = config_.nodeId, }, delayRespMessageBuffer_);
}

void ClockSync::start() {
    if (worker_.joinable()) {
        return;
    }
    stopping_ = false;
    deadline_ = Clock::now();
    sequence_ = 0;
    servo_.configure(config_.acquireBandwidth, config_.lockBandwidth);
    socket_.non_blocking(true);
    // Only reports that the socket option was accepted. Whether the stamps are
    // usable is decided by probation, and reported by reportStampMode().
    if (config_.useKernelspaceStamps) {
        (void)receiver_.enableKernelTimestamps();
    }
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

void ClockSync::onSample(std::function<void(const Sample&)> callback) {
    onSample_ = std::move(callback);
}

void ClockSync::stop() {
    asio::post(io_, [this] {
        stopping_ = true;
        socket_.close();
        timer_.cancel();
        io_.stop();   // backstop: run() returns even if a handler re-armed
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
    const bool gateSeeded = servo_.gateActive();
    statGate_.store(gateSeeded ? toSeconds(servo_.gateThreshold()) : 0.0,
                    std::memory_order_relaxed);
    statGateSeeded_.store(gateSeeded, std::memory_order_relaxed);
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
    if (statGateSeeded_.load(std::memory_order_relaxed)) {
        s.gateThreshold = statGate_.load(std::memory_order_relaxed);
    }
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

void ClockSync::armReceive() {
    if (stopping_) return;
    socket_.async_wait(asio::ip::udp::socket::wait_read,
        [this](const asio::error_code& ec) {
            if (stopping_) return;
            if (ec == asio::error::operation_aborted) return;
            if (ec) { armReceive(); return; }

            asio::error_code rec;
            const auto r = receiver_.receive(std::span{buffer_}, rec);
            if (!rec && r.bytes > 0) {
                if (!r.kernelStamp) userStamps_.fetch_add(1, std::memory_order_relaxed);
                remote_ = r.from;
                reportStampMode();          // one-shot, once probation settles
                handleReceive(r.bytes, r.stamp);
                publishStats();
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
            put_u64(delayRespMessageBuffer_ + kOffTargetId, msg->nodeId);
            put_u32(delayRespMessageBuffer_ + kOffSeq, sequence_++);
            put_u32(delayRespMessageBuffer_ + kOffRefSeq, msg->seq);
            put_u64(delayRespMessageBuffer_ + kOffT, toNanos(t));
            asio::error_code ec;
            socket_.send_to(asio::buffer(delayRespMessageBuffer_, kMessageBytes), config_.group, 0, ec);
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

            const auto mapping = servo_.mapping();

            if (mapping) {
                updateMapping(mapping->localRef, mapping->masterRef, mapping->skew);
                if (onSample_) {
                    onSample_(Sample{
                        .offset       = toSeconds(offset),
                        .delay        = toSeconds(delay),
                        .mappedOffset = -(mapping->masterRef - mapping->localRef),
                        .skew         = mapping->skew,
                        .rejected     = servo_.rejected(),
                        .tooSoon      = servo_.tooSoon(),
                    });
                }
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
    if (stopping_) return;
    deadline_ += config_.syncInterval;
    timer_.expires_at(deadline_);
    timer_.async_wait([this](const asio::error_code& ec) {
        if (stopping_) return;
        if (ec == asio::error::operation_aborted) return;
        sendSync();
        armSyncTimer();
    });
}

void ClockSync::sendSync() {
    put_u32(syncMessageBuffer_ + kOffSeq, sequence_++);
    const auto t = Clock::now();
    put_u64(syncMessageBuffer_ + kOffT, toNanos(t));
    asio::error_code ec;
    socket_.send_to(asio::buffer(syncMessageBuffer_, kMessageBytes), config_.group, 0, ec);
}

void ClockSync::armDelayReqTimer() {
    if (stopping_) return;
    deadline_ += config_.delayReqInterval;
    timer_.expires_at(deadline_);
    timer_.async_wait([this](const asio::error_code& ec) {
        if (stopping_) return;
        if (ec == asio::error::operation_aborted) return;
        sendDelayReq();
        armDelayReqTimer();
    });
}

void ClockSync::sendDelayReq() {
    const auto seq = sequence_++;
    put_u32(delayReqMessageBuffer_ + kOffSeq, seq);
    const auto t = Clock::now();
    put_u64(delayReqMessageBuffer_ + kOffT, toNanos(t));
    asio::error_code ec;
    socket_.send_to(asio::buffer(delayReqMessageBuffer_, kMessageBytes), config_.group, 0, ec);
    pending_ = {
        .t3 = sinceEpoch(t),
        .seq = seq,
        .valid = true
    };
}

}