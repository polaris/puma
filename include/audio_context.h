#ifndef AUDIO_CONTEXT_H
#define AUDIO_CONTEXT_H

#include <miniaudio.h>

class AudioContext {
public:
    AudioContext() = default;
    ~AudioContext() { uninit(); }

    AudioContext(const AudioContext&) = delete;
    AudioContext& operator=(const AudioContext&) = delete;

    bool init() {
        if (initialised_) {
            return false;
        }
        if (ma_context_init(nullptr, 0, nullptr, &context_) != MA_SUCCESS) {
            return false;
        }
        initialised_ = true;

        if (ma_context_get_devices(&context_, &playbackInfos_, &playbackCount_, &captureInfos_, &captureCount_) != MA_SUCCESS) {
            uninit();
            return false;
        }
        return true;
    }

    bool isInitialised() const {
        return initialised_;
    }

    ma_uint32 playbackCount() const {
        return playbackCount_;
    }

    ma_uint32 captureCount() const {
        return captureCount_;
    }

    const ma_device_info& playbackInfo(ma_uint32 index) const {
        return playbackInfos_[index];
    }

    const ma_device_info& captureInfo(ma_uint32 index) const {
        return captureInfos_[index];
    }

    ma_context* handle() {
        return &context_;
    }

private:
    void uninit() {
        if (!initialised_) {
            return;
        }

        ma_context_uninit(&context_);
        initialised_ = false;
        playbackInfos_ = nullptr;
        captureInfos_ = nullptr;
        playbackCount_ = 0;
        captureCount_ = 0;
    }

    ma_context context_{};
    ma_device_info* playbackInfos_ = nullptr;
    ma_device_info* captureInfos_  = nullptr;
    ma_uint32 playbackCount_ = 0;
    ma_uint32 captureCount_ = 0;
    bool initialised_ = false;
};

#endif  // AUDIO_CONTEXT_H
