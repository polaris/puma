#include "clock_sync.h"
#include "byte_order.h"

#include <iostream>

namespace clocksync {

ClockSync::ClockSync(Config config, Role role)
: socket_{io_}, timer_{io_}, config_{config}, role_{role}, deadline_{Clock::now()}, sequence_{0}, delayReqSeq_{0}, t1_{0}, t2_{0}, t3_{0} {
    net::configureBidirectional(socket_, config.group, config.iface, {.hops = 1, .loopback = config.loopback});
}

void ClockSync::start() {
    deadline_ = Clock::now();
    delayReqSeq_ = 0;
    sequence_ = 0;
    armReceive();
    if (role_ == Role::Master) {
        armSyncTimer();
    } else {
        armDelayReqTimer();
    }
    worker_ = std::thread([this] () {
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
}

[[nodiscard]] Stats ClockSync::stats() const noexcept {
}

void ClockSync::sendMessage(const SyncMessage& msg) {
    std::uint8_t buf[kMessageBytes];
    encode(msg, buf);
    asio::error_code ec;
    socket_.send_to(asio::buffer(buf, kMessageBytes), config_.group, 0, ec);
    if (ec) { /* TODO: count it, don't throw */ }
}

void ClockSync::armReceive() {
    socket_.async_receive_from(asio::buffer(buffer_), remote_,
        [this](const asio::error_code& ec, std::size_t n) {
            if (ec == asio::error::operation_aborted) {
                return;
            }
            if (ec) {
                armReceive();
                return;
            }
            const auto t = Clock::now();
            handleReceive(n, t);
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
                .t = nowNanos(),
            };
            sendMessage(out);
        }
    } else {
        if (msg->type == MsgType::DelayResp) {
            if (msg->targetId == config_.nodeId && msg->refSeq == delayReqSeq_) {
                const auto t4 = std::chrono::nanoseconds{static_cast<std::int64_t>(msg->t)};
                const auto a = t2_ - t1_;              // d + θ
                const auto b = t4 - t3_;              // d − θ
                const auto offset = (a - b) / 2;
                const auto delay  = (a + b) / 2;
                std::cout << toSeconds(offset) << " " << toSeconds(delay) << "\n";
            }
        } else if (msg->type == MsgType::Sync) {
            t1_ = std::chrono::nanoseconds{static_cast<std::int64_t>(msg->t)};
            t2_ = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch());
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
    SyncMessage msg{
        .type = MsgType::Sync,
        .domain = config_.domain,
        .nodeId = config_.nodeId,
        .seq = sequence_++,
        .t = nowNanos(),
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
    SyncMessage msg{
        .type = MsgType::DelayReq,
        .domain = config_.domain,
        .nodeId = config_.nodeId,
        .seq = sequence_++,
        .t = nowNanos(),
    };
    delayReqSeq_ = msg.seq;
    t3_ = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch());
    sendMessage(msg);
}

[[nodiscard]] double localToMaster(double local, const ClockMapping& mapping) noexcept {
    return mapping.masterRef + (local - mapping.localRef) * (1.0 + mapping.skew);
}

[[nodiscard]] double masterToLocal(double master, const ClockMapping& mapping) noexcept {
    return mapping.localRef + (master - mapping.masterRef) / (1.0 + mapping.skew);
}

}
