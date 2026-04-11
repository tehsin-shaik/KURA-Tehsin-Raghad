#include "whistle_detector/tflite_executer.hpp"

#if __has_include(<tflite_c_api.h>)
#include <tflite_c_api.h>
#elif __has_include(<tensorflow/lite/c/c_api.h>)
#include <tensorflow/lite/c/c_api.h>
#else
#error "TensorFlow Lite C API header not found"
#endif

#if defined(WHISTLE_DETECTOR_ENABLE_XNNPACK) && __has_include(<tflite_c_api_xnnpack_delegate.h>)
#include <tflite_c_api_xnnpack_delegate.h>
#define WHISTLE_DETECTOR_HAS_XNNPACK_HEADER 1
#endif

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace whistle_detector
{

#define MY_ASSERT_NE(v, e)                                                                            \
    do {                                                                                              \
        if ((v) == (e)) {                                                                             \
            std::fprintf(stderr, "%s (%s:%d): fatal assertion failed\\n", __func__, __FILE__, __LINE__); \
            std::fflush(stderr);                                                                      \
            std::abort();                                                                             \
        }                                                                                             \
    } while (0)

#define MY_ASSERT_EQ(v, e)                                                                            \
    do {                                                                                              \
        if ((v) != (e)) {                                                                             \
            std::fprintf(stderr, "%s (%s:%d): fatal assertion failed\\n", __func__, __FILE__, __LINE__); \
            std::fflush(stderr);                                                                      \
            std::abort();                                                                             \
        }                                                                                             \
    } while (0)

namespace
{

void cleanupInterpreter(TfLiteInterpreter *&interpreter, TfLiteDelegate *&delegate)
{
    if (interpreter != nullptr) {
        TfLiteInterpreterDelete(interpreter);
        interpreter = nullptr;
    }
#ifdef WHISTLE_DETECTOR_HAS_XNNPACK_HEADER
    if (delegate != nullptr) {
        TfLiteXNNPackDelegateDelete(delegate);
        delegate = nullptr;
    }
#else
    (void)delegate;
#endif
}

bool initializeInterpreter(
    TfLiteModel *model,
    const std::vector<int> &input_dims,
    int num_threads,
    bool use_xnnpack,
    TfLiteInterpreter *&interpreter,
    TfLiteDelegate *&delegate)
{
    cleanupInterpreter(interpreter, delegate);

    TfLiteInterpreterOptions *options = TfLiteInterpreterOptionsCreate();
    if (options == nullptr) {
        return false;
    }
    TfLiteInterpreterOptionsSetNumThreads(options, num_threads);

#ifdef WHISTLE_DETECTOR_HAS_XNNPACK_HEADER
    if (use_xnnpack) {
        TfLiteXNNPackDelegateOptions delegate_options = TfLiteXNNPackDelegateOptionsDefault();
        delegate_options.num_threads = num_threads;
        delegate = TfLiteXNNPackDelegateCreate(&delegate_options);
        if (delegate != nullptr) {
            TfLiteInterpreterOptionsAddDelegate(options, delegate);
        }
    }
#else
    (void)use_xnnpack;
#endif

    interpreter = TfLiteInterpreterCreate(model, options);
    TfLiteInterpreterOptionsDelete(options);
    if (interpreter == nullptr) {
        cleanupInterpreter(interpreter, delegate);
        return false;
    }

    if (TfLiteInterpreterResizeInputTensor(interpreter, 0, input_dims.data(), static_cast<int>(input_dims.size())) != kTfLiteOk ||
        TfLiteInterpreterAllocateTensors(interpreter) != kTfLiteOk ||
        TfLiteInterpreterGetInputTensorCount(interpreter) != 1 ||
        TfLiteInterpreterGetOutputTensorCount(interpreter) != 1) {
        cleanupInterpreter(interpreter, delegate);
        return false;
    }

    TfLiteTensor *input_tensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
    const TfLiteTensor *output_tensor = TfLiteInterpreterGetOutputTensor(interpreter, 0);
    if (input_tensor == nullptr || output_tensor == nullptr ||
        TfLiteTensorType(input_tensor) != kTfLiteFloat32 ||
        TfLiteTensorType(output_tensor) != kTfLiteFloat32) {
        cleanupInterpreter(interpreter, delegate);
        return false;
    }

    return true;
}

void finishInitialization(
    TfLiteModel *model,
    const std::vector<int> &input_dims,
    int num_threads,
    TfLiteInterpreter *&interpreter,
    TfLiteDelegate *&delegate)
{
#ifdef WHISTLE_DETECTOR_ENABLE_XNNPACK
    const bool initialized_with_delegate = initializeInterpreter(model, input_dims, num_threads, true, interpreter, delegate);
    if (!initialized_with_delegate) {
        const bool initialized_without_delegate = initializeInterpreter(model, input_dims, num_threads, false, interpreter, delegate);
        MY_ASSERT_EQ(initialized_without_delegate, true);
    }
#else
    const bool initialized = initializeInterpreter(model, input_dims, num_threads, false, interpreter, delegate);
    MY_ASSERT_EQ(initialized, true);
#endif
}

}  // namespace

TFLiteExecuter::~TFLiteExecuter()
{
    cleanupInterpreter(interpreter, delegate);
}

void TFLiteExecuter::loadModelFromFile(const std::string &file, const std::vector<int> &input_dims, int num_threads)
{
    TfLiteModel *model = TfLiteModelCreateFromFile(file.c_str());
    MY_ASSERT_NE(model, nullptr);
    finishInitialization(model, input_dims, num_threads, interpreter, delegate);
    TfLiteModelDelete(model);
}

void TFLiteExecuter::loadModelFromArray(const void *model_data, size_t length, const std::vector<int> &input_dims, int num_threads)
{
    TfLiteModel *model = TfLiteModelCreate(model_data, length);
    MY_ASSERT_NE(model, nullptr);
    finishInitialization(model, input_dims, num_threads, interpreter, delegate);
    TfLiteModelDelete(model);
}

float *TFLiteExecuter::getInputTensor()
{
    return TfLiteInterpreterGetInputTensor(interpreter, 0)->data.f;
}

const float *TFLiteExecuter::getOutputTensor() const
{
    return TfLiteInterpreterGetOutputTensor(interpreter, 0)->data.f;
}

size_t TFLiteExecuter::getElementsInputTensor() const
{
    return TfLiteTensorByteSize(TfLiteInterpreterGetInputTensor(interpreter, 0)) / sizeof(float);
}

size_t TFLiteExecuter::getElementsOutputTensor() const
{
    return TfLiteTensorByteSize(TfLiteInterpreterGetOutputTensor(interpreter, 0)) / sizeof(float);
}

void TFLiteExecuter::execute()
{
    MY_ASSERT_EQ(TfLiteInterpreterInvoke(interpreter), kTfLiteOk);
}

}  // namespace whistle_detector
