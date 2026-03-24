#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "whistle_detector/tflite_whistle_detection.hpp"

namespace whistle_detector
{
namespace
{
std::vector<int16_t> makeSineWave(double frequency_hz, double duration_seconds, double amplitude_scale)
{
    const auto total_samples = static_cast<size_t>(duration_seconds * TFLiteWhistleDetection::req_rate);
    std::vector<int16_t> samples(total_samples);
    for (size_t sample_idx = 0; sample_idx < total_samples; ++sample_idx) {
        const double t = static_cast<double>(sample_idx) / TFLiteWhistleDetection::req_rate;
        const double sample = std::sin(2.0 * M_PI * frequency_hz * t) * amplitude_scale * 32767.0;
        samples[sample_idx] = static_cast<int16_t>(sample);
    }
    return samples;
}

std::vector<int16_t> makeSilence(double duration_seconds)
{
    const auto total_samples = static_cast<size_t>(duration_seconds * TFLiteWhistleDetection::req_rate);
    return std::vector<int16_t>(total_samples, 0);
}

bool runDetector(
    TFLiteWhistleDetection &detector,
    const std::vector<int16_t> &samples,
    float &max_probability)
{
    bool detected = false;
    max_probability = 0.0f;

    for (size_t offset = 0; offset < samples.size(); offset += TFLiteWhistleDetection::req_frame_size) {
        const size_t end = std::min(samples.size(), offset + static_cast<size_t>(TFLiteWhistleDetection::req_frame_size));
        std::vector<int16_t> chunk(samples.begin() + offset, samples.begin() + end);
        float probability = 0.0f;
        detected = detector.process(chunk, probability) || detected;
        max_probability = std::max(max_probability, probability);
    }

    return detected;
}

std::string getModelDir()
{
    return WHISTLE_DETECTOR_TEST_DATA_DIR;
}
}  // namespace

TEST(TFLiteWhistleDetectionTest, DetectsWhistleLikeTone)
{
    TFLiteWhistleDetection detector(getModelDir());
    const auto whistle = makeSineWave(3000.0, 0.6, 0.95);

    float max_probability = 0.0f;
    const bool detected = runDetector(detector, whistle, max_probability);

    EXPECT_TRUE(detected);
    EXPECT_GT(max_probability, TFLiteWhistleDetection::prob_threshold);
}

TEST(TFLiteWhistleDetectionTest, RejectsSilenceAndOffBandTone)
{
    TFLiteWhistleDetection detector(getModelDir());
    const auto silence = makeSilence(0.6);

    float silence_probability = 0.0f;
    EXPECT_FALSE(runDetector(detector, silence, silence_probability));
    EXPECT_LT(silence_probability, TFLiteWhistleDetection::prob_threshold);

    detector.reset();

    const auto offband_tone = makeSineWave(4500.0, 0.6, 0.95);
    float offband_probability = 0.0f;
    EXPECT_FALSE(runDetector(detector, offband_tone, offband_probability));
    EXPECT_LT(offband_probability, TFLiteWhistleDetection::prob_threshold);
}

TEST(TFLiteWhistleDetectionTest, ResetClearsStreamState)
{
    TFLiteWhistleDetection detector(getModelDir());
    const auto whistle = makeSineWave(2500.0, 0.6, 0.95);

    float first_probability = 0.0f;
    EXPECT_TRUE(runDetector(detector, whistle, first_probability));
    EXPECT_GT(first_probability, TFLiteWhistleDetection::prob_threshold);

    detector.reset();

    float silence_probability = 0.0f;
    EXPECT_FALSE(runDetector(detector, makeSilence(0.6), silence_probability));
    EXPECT_LT(silence_probability, TFLiteWhistleDetection::prob_threshold);

    detector.reset();

    float second_probability = 0.0f;
    EXPECT_TRUE(runDetector(detector, whistle, second_probability));
    EXPECT_GT(second_probability, TFLiteWhistleDetection::prob_threshold);
}

}  // namespace whistle_detector
