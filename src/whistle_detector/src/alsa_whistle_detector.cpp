#include "whistle_detector/alsa_whistle_detector.hpp"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <utility>

namespace whistle_detector
{

namespace
{
constexpr size_t kLeftFrontMic = 0;
constexpr size_t kRightFrontMic = 1;
}

AlsaWhistleDetector::AlsaWhistleDetector(std::string model_dir, int num_threads)
    : model_dir_(std::move(model_dir)),
      whistle_detection_left_(model_dir_, num_threads),
      whistle_detection_right_(model_dir_, num_threads)
{
}

void AlsaWhistleDetector::setDetectionCallback(DetectionCallback callback)
{
    detection_callback_ = std::move(callback);
}

void AlsaWhistleDetector::requestStop()
{
    stop_requested_.store(true);
}

bool AlsaWhistleDetector::processChannels(const std::vector<int16_t> &interleaved, size_t frame_count, float &confidence)
{
    std::vector<std::vector<int16_t>> channels(
        TFLiteWhistleDetection::num_channels,
        std::vector<int16_t>(frame_count));

    for (size_t frame_idx = 0; frame_idx < frame_count; ++frame_idx) {
        for (size_t channel_idx = 0; channel_idx < TFLiteWhistleDetection::num_channels; ++channel_idx) {
            channels[channel_idx][frame_idx] = interleaved[frame_idx * TFLiteWhistleDetection::num_channels + channel_idx];
        }
    }

    float left_probability = 0.0f;
    float right_probability = 0.0f;
    const bool left_detected = whistle_detection_left_.process(channels[kLeftFrontMic], left_probability);
    const bool right_detected = whistle_detection_right_.process(channels[kRightFrontMic], right_probability);
    confidence = std::max(left_probability, right_probability);
    return left_detected || right_detected;
}

void AlsaWhistleDetector::run(
    const std::string &device_name,
    std::chrono::milliseconds cooldown,
    float publish_confidence_threshold)
{
    snd_pcm_t *handle = nullptr;
    int rc = snd_pcm_open(&handle, device_name.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) {
        throw std::runtime_error("unable to open pcm device: " + std::string(snd_strerror(rc)));
    }

    snd_pcm_hw_params_t *params = nullptr;
    snd_pcm_hw_params_alloca(&params);

    auto cleanup = [&]() {
        if (handle != nullptr) {
            snd_pcm_drain(handle);
            snd_pcm_close(handle);
        }
    };

    rc = snd_pcm_hw_params_any(handle, params);
    if (rc < 0) {
        cleanup();
        throw std::runtime_error("snd_pcm_hw_params_any failed: " + std::string(snd_strerror(rc)));
    }

    rc = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (rc < 0) {
        cleanup();
        throw std::runtime_error("snd_pcm_hw_params_set_access failed: " + std::string(snd_strerror(rc)));
    }

    rc = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
    if (rc < 0) {
        cleanup();
        throw std::runtime_error("snd_pcm_hw_params_set_format failed: " + std::string(snd_strerror(rc)));
    }

    rc = snd_pcm_hw_params_set_channels(handle, params, TFLiteWhistleDetection::num_channels);
    if (rc < 0) {
        cleanup();
        throw std::runtime_error("snd_pcm_hw_params_set_channels failed: " + std::string(snd_strerror(rc)));
    }

    unsigned int req_rate = TFLiteWhistleDetection::req_rate;
    snd_pcm_uframes_t req_frame_size = TFLiteWhistleDetection::req_frame_size;
    int dir = 0;

    rc = snd_pcm_hw_params_set_rate_near(handle, params, &req_rate, &dir);
    if (rc < 0) {
        cleanup();
        throw std::runtime_error("snd_pcm_hw_params_set_rate_near failed: " + std::string(snd_strerror(rc)));
    }

    rc = snd_pcm_hw_params_set_period_size_near(handle, params, &req_frame_size, &dir);
    if (rc < 0) {
        cleanup();
        throw std::runtime_error("snd_pcm_hw_params_set_period_size_near failed: " + std::string(snd_strerror(rc)));
    }

    rc = snd_pcm_hw_params(handle, params);
    if (rc < 0) {
        cleanup();
        throw std::runtime_error("snd_pcm_hw_params failed: " + std::string(snd_strerror(rc)));
    }

    snd_pcm_nonblock(handle, 1);

    std::vector<int16_t> buffer(req_frame_size * TFLiteWhistleDetection::num_channels);
    auto last_publish = std::chrono::steady_clock::time_point::min();

    while (!stop_requested_.load()) {
        rc = snd_pcm_readi(handle, buffer.data(), req_frame_size);
        if (rc == -EPIPE) {
            snd_pcm_prepare(handle);
            continue;
        }
        if (rc == -EAGAIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (rc < 0) {
            cleanup();
            throw std::runtime_error("snd_pcm_readi failed: " + std::string(snd_strerror(rc)));
        }
        if (rc == 0) {
            continue;
        }

        float confidence = 0.0f;
        const bool detected = processChannels(buffer, static_cast<size_t>(rc), confidence);
        if (!detected || confidence < publish_confidence_threshold) {
            continue;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_publish < cooldown) {
            continue;
        }
        last_publish = now;

        if (detection_callback_) {
            detection_callback_(confidence);
        }
    }

    cleanup();
}

}  // namespace whistle_detector
