#ifndef WAV_WRITER_H
#define WAV_WRITER_H

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>

// Minimal 16-bit PCM RIFF/WAVE writer. The header goes out with zeroed sizes
// and is patched in close(), so a file is only valid once closed. Little-endian
// hosts only, which is every platform this runs on.
class WavWriter {
public:
    ~WavWriter() { close(); }

    bool open(const std::string& path, std::uint16_t channels, std::uint32_t sampleRate) {
        if (channels == 0) return false;
        out_.open(path, std::ios::binary | std::ios::trunc);
        if (!out_) return false;

        const std::uint32_t fmtSize    = 16;
        const std::uint16_t pcm        = 1;
        const std::uint16_t bits       = 16;
        const std::uint16_t blockAlign = static_cast<std::uint16_t>(channels * bits / 8);
        const std::uint32_t byteRate   = sampleRate * blockAlign;

        // The RIFF and data sizes are 32-bit, so the payload cannot exceed
        // 4 GiB minus the 36 bytes the riff size covers ahead of it. Round down
        // to a whole frame so the file never ends mid-frame.
        maxDataBytes_ = ((0xFFFFFFFFu - 36u) / blockAlign) * blockAlign;

        std::array<std::uint8_t, 44> h{};
        std::memcpy(h.data() +  0, "RIFF", 4);          // + 4: riff size, patched
        std::memcpy(h.data() +  8, "WAVEfmt ", 8);
        std::memcpy(h.data() + 16, &fmtSize, 4);
        std::memcpy(h.data() + 20, &pcm, 2);
        std::memcpy(h.data() + 22, &channels, 2);
        std::memcpy(h.data() + 24, &sampleRate, 4);
        std::memcpy(h.data() + 28, &byteRate, 4);
        std::memcpy(h.data() + 32, &blockAlign, 2);
        std::memcpy(h.data() + 34, &bits, 2);
        std::memcpy(h.data() + 36, "data", 4);          // + 40: data size, patched
        out_.write(reinterpret_cast<const char*>(h.data()), h.size());
        return static_cast<bool>(out_);
    }

    bool isOpen() const { return out_.is_open(); }

    // Returns false if the write was refused because the file is full. Partial
    // writes are never made: the caller loses at most one period, not a frame.
    bool write(const void* pcm, std::size_t bytes) {
        if (!out_.is_open() || full_) return false;
        if (bytes > maxDataBytes_ - dataBytes_) {
            full_ = true;
            return false;
        }
        out_.write(static_cast<const char*>(pcm), static_cast<std::streamsize>(bytes));
        dataBytes_ += static_cast<std::uint32_t>(bytes);
        return true;
    }

    // True once the 4 GiB RIFF ceiling is reached; what is on disk stays valid.
    bool full() const { return full_; }

    std::uint32_t dataBytes() const { return dataBytes_; }

    void close() {
        if (!out_.is_open()) return;
        const std::uint32_t riffSize = 36 + dataBytes_;
        out_.seekp(4);
        out_.write(reinterpret_cast<const char*>(&riffSize), 4);
        out_.seekp(40);
        out_.write(reinterpret_cast<const char*>(&dataBytes_), 4);
        out_.close();
    }

private:
    std::ofstream out_;
    std::uint32_t dataBytes_    = 0;
    std::uint32_t maxDataBytes_ = 0;
    bool          full_         = false;
};

#endif  // WAV_WRITER_H
