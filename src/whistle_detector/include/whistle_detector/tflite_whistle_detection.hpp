#pragma once

#include <array>
#include <complex>
#include <mutex>
#include <string>
#include <vector>

#include "whistle_detector/tflite_executer.hpp"

namespace whistle_detector
{

class TFLiteWhistleDetection
{
public:
    explicit TFLiteWhistleDetection(const std::string &model_dir);
    constexpr static uint32_t req_rate = 44100;
    constexpr static uint32_t req_frame_size = 441;
    constexpr static uint32_t num_channels = 2;

    constexpr static size_t frame_size = 384;
    constexpr static int ffts_per_patch = 12;
    constexpr static int patch_height = 64;
    constexpr static int expected_number_of_elements = ffts_per_patch * patch_height;
    constexpr static float prob_threshold = 0.998600f;

    bool process(const std::vector<int16_t> &vals, float &max_probability);
    void reset();

private:
    void computeSpectrum(const std::vector<int16_t> &frame, std::array<std::complex<float>, patch_height> &spectrum) const;

    int c_zero_current_patch_fft = 0;
    TFLiteExecuter classifier_zero_;

    int c_half_start_offset = ffts_per_patch / 2;
    int c_half_countdown_to_start = c_half_start_offset;
    int c_half_current_patch_fft = 0;
    TFLiteExecuter classifier_half_;

    float *input_c_zero_ = nullptr;
    float *input_c_half_ = nullptr;

    std::vector<int16_t> buffer_;
    std::array<std::array<float, frame_size>, patch_height> cos_table_{};
    std::array<std::array<float, frame_size>, patch_height> sin_table_{};
    std::mutex mtx_;
};

}  // namespace whistle_detector
