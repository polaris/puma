#ifndef AUDIO_RECORDER_H
#define AUDIO_RECORDER_H

#include <miniaudio.h>

#include "audio_context.h"

class AudioRecorder {
public:
    using DataCallback = void (*)(void* user, const void* input, ma_uint32 frameCount);

    struct Config {
        ma_format format = ma_format_s16;
        ma_uint32 channels = 0;  // 0 selects the device's native count
        ma_uint32 sampleRate = 0;  // 0 selects the device's native rate
        ma_uint32 periodSizeInFrames = 0;
        ma_uint32 periods = 0;
        ma_performance_profile performanceProfile = ma_performance_profile_low_latency;
    };

    AudioRecorder() = default;
    ~AudioRecorder() { close(); }

    AudioRecorder(const AudioRecorder&) = delete;
    AudioRecorder& operator=(const AudioRecorder&) = delete;
    AudioRecorder(AudioRecorder&&) = delete;
    AudioRecorder& operator=(AudioRecorder&&) = delete;

    // `id` only has to stay alive for this call: miniaudio copies it into the
    // device. A null callback discards whatever is captured.
    bool open(AudioContext& context, const ma_device_id& id, const Config& config,
              DataCallback callback = nullptr, void* user = nullptr) {
        if (opened_) return false;

        callback_ = callback;
        user_ = user;

        ma_device_config deviceConfig = ma_device_config_init(ma_device_type_capture);
        deviceConfig.capture.pDeviceID = &id;
        deviceConfig.capture.format = config.format;
        deviceConfig.capture.channels = config.channels;
        deviceConfig.sampleRate = config.sampleRate;
        deviceConfig.periodSizeInFrames = config.periodSizeInFrames;
        deviceConfig.periods = config.periods;
        deviceConfig.performanceProfile = config.performanceProfile;
        deviceConfig.dataCallback = &dataCallback;
        deviceConfig.pUserData = this;

        if (ma_device_init(context.handle(), &deviceConfig, &device_) != MA_SUCCESS) return false;
        opened_ = true;
        return true;
    }

    // The audio thread exists from here on. Call it once everything downstream
    // of the callback is in place, and keep it the last step of setup.
    bool start() {
        if (!opened_ || started_) return false;
        if (ma_device_start(&device_) != MA_SUCCESS) return false;
        started_ = true;
        return true;
    }

    // Returns once the audio thread is no longer in the callback.
    void stop() {
        if (!started_) return;
        ma_device_stop(&device_);
        started_ = false;
    }

    void close() {
        stop();
        if (!opened_) return;
        ma_device_uninit(&device_);
        opened_ = false;
    }

    bool isOpen() const { return opened_; }
    bool isStarted() const { return started_; }

    // Valid once open(): what the device actually negotiated, which is not
    // necessarily what Config asked for.
    ma_uint32 sampleRate() const { return device_.sampleRate; }

    // The rate the hardware actually runs at. sampleRate() is only the rate the
    // callback is fed at: miniaudio accepts whatever was asked for and resamples
    // to reach this one, so these two differing means a resampler is in the path.
    ma_uint32 internalSampleRate() const { return device_.capture.internalSampleRate; }

    ma_uint32 channels() const { return device_.capture.channels; }
    ma_format format() const { return device_.capture.format; }
    ma_uint32 bytesPerFrame() const {
        return ma_get_bytes_per_frame(device_.capture.format, device_.capture.channels);
    }

private:
    static void dataCallback(ma_device* device, void* output, const void* input,
                             ma_uint32 frameCount) {
        (void)output;
        auto* self = static_cast<AudioRecorder*>(device->pUserData);
        if (self->callback_ == nullptr) return;
        self->callback_(self->user_, input, frameCount);
    }

    ma_device device_{};
    DataCallback callback_ = nullptr;
    void* user_ = nullptr;
    bool opened_ = false;
    bool started_ = false;
};

#endif  // AUDIO_RECORDER_H
