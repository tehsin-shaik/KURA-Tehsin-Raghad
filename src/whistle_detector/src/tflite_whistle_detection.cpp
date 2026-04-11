#include "whistle_detector/tflite_whistle_detection.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace whistle_detector
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TFLiteWhistleDetection::TFLiteWhistleDetection(const std::string &model_dir, int num_threads)
#ifdef WHISTLE_DETECTOR_USE_FFTW
    : fft_real_buffer_(frame_size),
      fft_complex_buffer_(fft_spec_size)
#endif
{
    const std::string model_path = model_dir + "/whistle-classifier.tflite";
    const int effective_threads = std::max(1, num_threads);
    classifier_zero_.loadModelFromFile(model_path, {1, ffts_per_patch, patch_height, 1}, effective_threads);
    classifier_half_.loadModelFromFile(model_path, {1, ffts_per_patch, patch_height, 1}, effective_threads);

    input_c_zero_ = classifier_zero_.getInputTensor();
    input_c_half_ = classifier_half_.getInputTensor();

#ifdef WHISTLE_DETECTOR_USE_FFTW
    fft_plan_ = fftwf_plan_dft_r2c_1d(frame_size, fft_real_buffer_.data(), fft_complex_buffer_.data(), FFTW_ESTIMATE);
    if (fft_plan_ == nullptr) {
        throw std::runtime_error("failed to create FFTW plan for whistle detection");
    }
#else
    for (size_t k = 0; k < patch_height; ++k) {
        for (size_t n = 0; n < frame_size; ++n) {
            const float angle = static_cast<float>(2.0 * kPi * k * n / frame_size);
            cos_table_[k][n] = std::cos(angle);
            sin_table_[k][n] = std::sin(angle);
        }
    }
#endif
}

TFLiteWhistleDetection::~TFLiteWhistleDetection()
{
#ifdef WHISTLE_DETECTOR_USE_FFTW
    if (fft_plan_ != nullptr) {
        fftwf_destroy_plan(fft_plan_);
    }
#endif
}

#ifndef WHISTLE_DETECTOR_USE_FFTW
void TFLiteWhistleDetection::computeSpectrum(
    const std::vector<int16_t> &frame,
    std::array<std::complex<float>, patch_height> &spectrum) const
{
    for (size_t k = 0; k < patch_height; ++k) {
        float real = 0.0f;
        float imag = 0.0f;
        for (size_t n = 0; n < frame_size; ++n) {
            const float sample = static_cast<float>(frame[n]);
            real += sample * cos_table_[k][n];
            imag -= sample * sin_table_[k][n];
        }
        spectrum[k] = {real, imag};
    }
}
#endif

bool TFLiteWhistleDetection::process(const std::vector<int16_t> &vals, float &max_probability)
{
    std::lock_guard<std::mutex> lock(mtx_);
    buffer_.insert(buffer_.end(), vals.begin(), vals.end());

    size_t c_zero_offset = static_cast<size_t>(c_zero_current_patch_fft) * patch_height;
    size_t c_half_offset = static_cast<size_t>(c_half_current_patch_fft) * patch_height;

    bool c_zero_result = false;
    bool c_half_result = false;
    max_probability = 0.0f;
#ifndef WHISTLE_DETECTOR_USE_FFTW
    std::array<std::complex<float>, patch_height> spectrum{};
#endif

    while (buffer_.size() >= frame_size) {
#ifdef WHISTLE_DETECTOR_USE_FFTW
        std::copy_n(buffer_.begin(), frame_size, fft_real_buffer_.begin());
        fftwf_execute(fft_plan_);
#else
        std::vector<int16_t> frame(buffer_.begin(), buffer_.begin() + static_cast<std::vector<int16_t>::difference_type>(frame_size));
        computeSpectrum(frame, spectrum);
#endif

        for (int i = 0; i < patch_height; ++i) {
#ifdef WHISTLE_DETECTOR_USE_FFTW
            fftwf_complex &c = fft_complex_buffer_[static_cast<size_t>(i)];
            float tmp = std::sqrt(c[0] * c[0] + c[1] * c[1]);
#else
            const auto &c = spectrum[static_cast<size_t>(i)];
            float tmp = std::abs(c);
#endif
            tmp = std::log(tmp) / 15.0f;

            if (std::isnan(tmp) || std::isinf(tmp)) {
                tmp = 0.0f;
            }

            input_c_zero_[c_zero_offset + static_cast<size_t>(i)] = tmp;
            input_c_half_[c_half_offset + static_cast<size_t>(i)] = tmp;
        }

        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::vector<int16_t>::difference_type>(frame_size));

        ++c_zero_current_patch_fft;
        if (c_zero_current_patch_fft == ffts_per_patch) {
            c_zero_current_patch_fft = 0;
            classifier_zero_.execute();
            const float probability = *classifier_zero_.getOutputTensor();
            max_probability = std::max(max_probability, probability);
            c_zero_result = c_zero_result || probability > prob_threshold;
        }

        --c_half_countdown_to_start;
        if (c_half_countdown_to_start <= 0) {
            c_half_countdown_to_start = 0;
            ++c_half_current_patch_fft;
            if (c_half_current_patch_fft == ffts_per_patch) {
                c_half_current_patch_fft = 0;
                classifier_half_.execute();
                const float probability = *classifier_half_.getOutputTensor();
                max_probability = std::max(max_probability, probability);
                c_half_result = c_half_result || probability > prob_threshold;
            }
        }

        c_zero_offset = static_cast<size_t>(c_zero_current_patch_fft) * patch_height;
        c_half_offset = static_cast<size_t>(c_half_current_patch_fft) * patch_height;
    }

    return c_zero_result || c_half_result;
}

void TFLiteWhistleDetection::reset()
{
    std::lock_guard<std::mutex> lock(mtx_);
    c_zero_current_patch_fft = 0;
    c_half_current_patch_fft = 0;
    c_half_countdown_to_start = c_half_start_offset;
    buffer_.clear();
}

}  // namespace whistle_detector
