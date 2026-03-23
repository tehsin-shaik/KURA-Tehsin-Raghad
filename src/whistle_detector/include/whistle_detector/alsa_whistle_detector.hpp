#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "whistle_detector/tflite_whistle_detection.hpp"

namespace whistle_detector
{

class AlsaWhistleDetector
{
public:
    using DetectionCallback = std::function<void(float confidence)>;

    explicit AlsaWhistleDetector(std::string model_dir);

    void setDetectionCallback(DetectionCallback callback);

    void requestStop();

    void run(
        const std::string &device_name,
        std::chrono::milliseconds cooldown,
        float publish_confidence_threshold);

private:
    bool processChannels(const std::vector<int16_t> &interleaved, size_t frame_count, float &confidence);

    std::atomic<bool> stop_requested_{false};
    DetectionCallback detection_callback_;
    std::string model_dir_;
    TFLiteWhistleDetection whistle_detection_left_;
    TFLiteWhistleDetection whistle_detection_right_;
};

}  // namespace whistle_detector
