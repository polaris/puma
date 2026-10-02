#ifndef PRESENTATION_TIMELINE_H
#define PRESENTATION_TIMELINE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

// One per packet, from the network thread to the audio thread: frame `k`
// (a ring position) is to leave the speaker at master time `t`.
struct NetReport {
    double t = 0.0;             // the packet's stamp: master time, seconds since the steady epoch
    std::uint64_t k = 0;        // ring position of the packet's first frame
    bool restart = false;       // a new stream: earlier reports no longer apply
};

// The stream's presentation times, as the audio thread sees them: which ring
// position is due at a given master time.
//
// Packets arrive about L ahead of their presentation time, so the newest
// reports lie well in the future of the time asked about. Extrapolating back
// from the last two would multiply the stamps' jitter by L over a packet
// period; this keeps a history instead and interpolates between the two
// reports around the time asked about (the paper's equation (1)).
class PresentationTimeline {
public:
    // About 700 ms of 240-frame packets at 44.1 kHz: more than the ring holds.
    static constexpr std::size_t kCapacity = 128;

    void clear() noexcept {
        head_ = 0;
        count_ = 0;
    }

    void add(const NetReport& report) noexcept {
        if (report.restart) {
            clear();
        }
        if (count_ == kCapacity) {
            pop();      // the oldest; only extrapolation back in time needs it
        }
        reports_[(head_ + count_) % kCapacity] = report;
        ++count_;
    }

    // How far, in frames, the frame due at master time `m` lies ahead of ring
    // position `k`; fractional. Nothing before two reports. Forgets reports
    // older than the pair around `m`, so `m` should not go back in time.
    [[nodiscard]] std::optional<double> framesAhead(double m, std::uint64_t k) noexcept {
        while (count_ > 2 && at(1).t <= m) {
            pop();
        }
        if (count_ < 2) {
            return std::nullopt;
        }
        // Between the two, or beyond them on either side: before the oldest
        // report at the start, past the newest when the stream stops.
        const NetReport& a = at(0);
        const NetReport& b = at(1);
        const double span = b.t - a.t;
        if (!(span > 0.0)) {
            return std::nullopt;
        }
        // Integer subtraction first: the positions are too large for a double.
        return static_cast<double>(static_cast<std::int64_t>(a.k - k))
             + static_cast<double>(b.k - a.k) * (m - a.t) / span;
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return count_;
    }

private:
    [[nodiscard]] const NetReport& at(std::size_t i) const noexcept {
        return reports_[(head_ + i) % kCapacity];
    }

    void pop() noexcept {
        head_ = (head_ + 1) % kCapacity;
        --count_;
    }

    std::array<NetReport, kCapacity> reports_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
};

#endif  // PRESENTATION_TIMELINE_H
