#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#ifndef WHISTLE_DETECTOR_TFLITE_UNAVAILABLE
#include "whistle_detector/tflite_whistle_detection.hpp"
#endif

namespace whistle_detector
{

class AlsaWhistleDetector
{
public:
    using DetectionCallback = std::function<void(float confidence)>;

    explicit AlsaWhistleDetector(std::string model_dir, int num_threads = 1);

    void setDetectionCallback(DetectionCallback callback);

    void requestStop();

    void run(
        const std::string &device_name,
        std::chrono::milliseconds cooldown,
        float publish_confidence_threshold);

private:
#ifndef WHISTLE_DETECTOR_TFLITE_UNAVAILABLE
    bool processChannels(const std::vector<int16_t> &interleaved, size_t frame_count, float &confidence);
#endif

    std::atomic<bool> stop_requested_{false};
    DetectionCallback detection_callback_;
    std::string model_dir_;
#ifndef WHISTLE_DETECTOR_TFLITE_UNAVAILABLE
    TFLiteWhistleDetection whistle_detection_left_;
    TFLiteWhistleDetection whistle_detection_right_;
#endif
};

}  // namespace whistle_detector
