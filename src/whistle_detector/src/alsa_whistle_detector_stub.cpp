#include "whistle_detector/alsa_whistle_detector.hpp"

#include <chrono>
#include <cstdio>
#include <thread>
#include <utility>

namespace whistle_detector
{

AlsaWhistleDetector::AlsaWhistleDetector(std::string model_dir, int num_threads)
    : model_dir_(std::move(model_dir))
{
    (void)num_threads;
}

void AlsaWhistleDetector::setDetectionCallback(DetectionCallback callback)
{
    detection_callback_ = std::move(callback);
}

void AlsaWhistleDetector::requestStop()
{
    stop_requested_.store(true);
}

void AlsaWhistleDetector::run(
    const std::string &device_name,
    std::chrono::milliseconds cooldown,
    float publish_confidence_threshold)
{
    (void)device_name;
    (void)cooldown;
    (void)publish_confidence_threshold;

    std::fprintf(
        stderr,
        "whistle_detector: TensorFlow Lite backend is unavailable on this architecture; whistle detection is disabled.\n");

    while (!stop_requested_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

}  // namespace whistle_detector
