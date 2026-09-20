#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include <miniaudio.h>

#include "audio_context.h"

class AudioPlayer {
public:
    using DataCallback = void (*)(void* user, void* output, ma_uint32 frameCount);

    struct Config {
        ma_format format = ma_format_s16;
        ma_uint32 channels = 0;  // 0 selects the device's native count
        ma_uint32 sampleRate = 0;  // 0 selects the device's native rate
        ma_uint32 periodSizeInFrames = 0;
        ma_uint32 periods = 0;
        ma_performance_profile performanceProfile = ma_performance_profile_low_latency;
    };

    AudioPlayer() = default;
    ~AudioPlayer() {
        close();
    }

    AudioPlayer(const AudioPlayer&) = delete;
    AudioPlayer& operator=(const AudioPlayer&) = delete;
    AudioPlayer(AudioPlayer&&) = delete;
    AudioPlayer& operator=(AudioPlayer&&) = delete;

    bool open(AudioContext& context, const ma_device_id& id, const Config& config, DataCallback callback = nullptr, void* user = nullptr) {
        if (opened_) {
            return false;
        }

        callback_ = callback;
        user_     = user;

        ma_device_config deviceConfig = ma_device_config_init(ma_device_type_playback);
        deviceConfig.playback.pDeviceID = &id;
        deviceConfig.playback.format = config.format;
        deviceConfig.playback.channels = config.channels;
        deviceConfig.sampleRate = config.sampleRate;
        deviceConfig.periodSizeInFrames = config.periodSizeInFrames;
        deviceConfig.periods = config.periods;
        deviceConfig.performanceProfile = config.performanceProfile;
        deviceConfig.dataCallback = &dataCallback;
        deviceConfig.pUserData = this;

        if (ma_device_init(context.handle(), &deviceConfig, &device_) != MA_SUCCESS) {
            return false;
        }

        opened_ = true;
        return true;
    }

    bool start() {
        if (!opened_ || started_) {
            return false;
        }
        if (ma_device_start(&device_) != MA_SUCCESS) {
            return false;
        }
        started_ = true;
        return true;
    }

    // Returns once the audio thread is no longer in the callback.
    void stop() {
        if (!started_) {
            return;
        }
        ma_device_stop(&device_);
        started_ = false;
    }

    void close() {
        stop();
        if (!opened_) {
            return;
        }
        ma_device_uninit(&device_);
        opened_ = false;
    }

    bool isOpen() const {
        return opened_;
    }

    bool isStarted() const {
        return started_;
    }

    ma_uint32 sampleRate() const {
        return device_.sampleRate;
    }

    ma_uint32 internalSampleRate() const {
        return device_.playback.internalSampleRate;
    }

    ma_uint32 channels() const {
        return device_.playback.channels;
    }

    ma_format format() const {
        return device_.playback.format;
    }

    ma_uint32 bytesPerFrame() const {
        return ma_get_bytes_per_frame(device_.playback.format, device_.playback.channels);
    }

private:
    static void dataCallback(ma_device* device, void* output, const void* input, ma_uint32 frameCount) {
        (void)input;
        auto* self = static_cast<AudioPlayer*>(device->pUserData);
        if (self->callback_ != nullptr) {
            self->callback_(self->user_, output, frameCount);
            return;
        }
        ma_silence_pcm_frames(output, frameCount, device->playback.format, device->playback.channels);
    }

    ma_device device_{};
    DataCallback callback_ = nullptr;
    void* user_ = nullptr;
    bool opened_ = false;
    bool started_ = false;
};

#endif  // AUDIO_PLAYER_H
