#pragma once

#ifdef WHISTLE_DETECTOR_USE_FFTW
#include <fftw3.h>
#endif

#include <array>
#include <complex>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#ifdef WHISTLE_DETECTOR_USE_FFTW
#include "whistle_detector/fftw_allocator.h"
#endif
#include "whistle_detector/tflite_executer.hpp"

namespace whistle_detector
{

class TFLiteWhistleDetection
{
public:
    explicit TFLiteWhistleDetection(const std::string &model_dir, int num_threads = 1);
    ~TFLiteWhistleDetection();

    constexpr static uint32_t req_rate = 44100;
    constexpr static uint32_t req_frame_size = 441;
    constexpr static uint32_t num_channels = 2;

    constexpr static size_t frame_size = 384;
    constexpr static int ffts_per_patch = 12;
    constexpr static int patch_height = 64;
    constexpr static size_t fft_spec_size = frame_size / 2 + 1;
    constexpr static int expected_number_of_elements = ffts_per_patch * patch_height;
    constexpr static float prob_threshold = 0.998600f;

    bool process(const std::vector<int16_t> &vals, float &max_probability);
    void reset();

private:
#ifndef WHISTLE_DETECTOR_USE_FFTW
    void computeSpectrum(const std::vector<int16_t> &frame, std::array<std::complex<float>, patch_height> &spectrum) const;
#endif

    int c_zero_current_patch_fft = 0;
    TFLiteExecuter classifier_zero_;

    int c_half_start_offset = ffts_per_patch / 2;
    int c_half_countdown_to_start = c_half_start_offset;
    int c_half_current_patch_fft = 0;
    TFLiteExecuter classifier_half_;

    float *input_c_zero_ = nullptr;
    float *input_c_half_ = nullptr;

    std::vector<int16_t> buffer_;
#ifdef WHISTLE_DETECTOR_USE_FFTW
    fftwf_vector<float> fft_real_buffer_;
    fftwf_vector<fftwf_complex> fft_complex_buffer_;
    fftwf_plan fft_plan_ = nullptr;
#else
    std::array<std::array<float, frame_size>, patch_height> cos_table_{};
    std::array<std::array<float, frame_size>, patch_height> sin_table_{};
#endif
    std::mutex mtx_;
};

}  // namespace whistle_detector
