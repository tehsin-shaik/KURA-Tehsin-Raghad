#include "whistle_detector/tflite_executer.hpp"

#include <tflite_c_api.h>
#include <tflite_c_api_xnnpack_delegate.h>

#include <cstdlib>
#include <cstdio>

namespace whistle_detector
{

#define MY_ASSERT_NE(v, e)                                                                            \
    do {                                                                                              \
        if ((v) == (e)) {                                                                             \
            std::fprintf(stderr, "%s (%s:%d): fatal assertion failed\n", __func__, __FILE__, __LINE__); \
            std::fflush(stderr);                                                                      \
            std::abort();                                                                             \
        }                                                                                             \
    } while (0)

#define MY_ASSERT_EQ(v, e)                                                                            \
    do {                                                                                              \
        if ((v) != (e)) {                                                                             \
            std::fprintf(stderr, "%s (%s:%d): fatal assertion failed\n", __func__, __FILE__, __LINE__); \
            std::fflush(stderr);                                                                      \
            std::abort();                                                                             \
        }                                                                                             \
    } while (0)

TFLiteExecuter::~TFLiteExecuter()
{
    if (interpreter != nullptr) {
        TfLiteInterpreterDelete(interpreter);
    }
    if (delegate != nullptr) {
        TfLiteXNNPackDelegateDelete(delegate);
    }
}

void TFLiteExecuter::loadModelFromFile(const std::string &file, const std::vector<int> &input_dims, int num_threads)
{
    TfLiteModel *model = TfLiteModelCreateFromFile(file.c_str());
    MY_ASSERT_NE(model, nullptr);

    TfLiteXNNPackDelegateOptions delegate_options = TfLiteXNNPackDelegateOptionsDefault();
    delegate_options.num_threads = num_threads;
    delegate = TfLiteXNNPackDelegateCreate(&delegate_options);
    MY_ASSERT_NE(delegate, nullptr);

    TfLiteInterpreterOptions *options = TfLiteInterpreterOptionsCreate();
    MY_ASSERT_NE(options, nullptr);
    TfLiteInterpreterOptionsSetNumThreads(options, num_threads);
    TfLiteInterpreterOptionsAddDelegate(options, delegate);

    interpreter = TfLiteInterpreterCreate(model, options);
    MY_ASSERT_NE(interpreter, nullptr);

    TfLiteInterpreterOptionsDelete(options);
    TfLiteModelDelete(model);

    MY_ASSERT_EQ(TfLiteInterpreterResizeInputTensor(interpreter, 0, input_dims.data(), static_cast<int>(input_dims.size())), kTfLiteOk);
    MY_ASSERT_EQ(TfLiteInterpreterAllocateTensors(interpreter), kTfLiteOk);
    MY_ASSERT_EQ(TfLiteInterpreterGetInputTensorCount(interpreter), 1);
    MY_ASSERT_EQ(TfLiteInterpreterGetOutputTensorCount(interpreter), 1);

    TfLiteTensor *input_tensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
    MY_ASSERT_NE(input_tensor, nullptr);
    MY_ASSERT_EQ(TfLiteTensorType(input_tensor), kTfLiteFloat32);

    const TfLiteTensor *output_tensor = TfLiteInterpreterGetOutputTensor(interpreter, 0);
    MY_ASSERT_NE(output_tensor, nullptr);
    MY_ASSERT_EQ(TfLiteTensorType(output_tensor), kTfLiteFloat32);
}

void TFLiteExecuter::loadModelFromArray(const void *model_data, size_t length, const std::vector<int> &input_dims, int num_threads)
{
    TfLiteModel *model = TfLiteModelCreate(model_data, length);
    MY_ASSERT_NE(model, nullptr);

    TfLiteXNNPackDelegateOptions delegate_options = TfLiteXNNPackDelegateOptionsDefault();
    delegate_options.num_threads = num_threads;
    delegate = TfLiteXNNPackDelegateCreate(&delegate_options);
    MY_ASSERT_NE(delegate, nullptr);

    TfLiteInterpreterOptions *options = TfLiteInterpreterOptionsCreate();
    MY_ASSERT_NE(options, nullptr);
    TfLiteInterpreterOptionsSetNumThreads(options, num_threads);
    TfLiteInterpreterOptionsAddDelegate(options, delegate);

    interpreter = TfLiteInterpreterCreate(model, options);
    MY_ASSERT_NE(interpreter, nullptr);

    TfLiteInterpreterOptionsDelete(options);
    TfLiteModelDelete(model);

    MY_ASSERT_EQ(TfLiteInterpreterResizeInputTensor(interpreter, 0, input_dims.data(), static_cast<int>(input_dims.size())), kTfLiteOk);
    MY_ASSERT_EQ(TfLiteInterpreterAllocateTensors(interpreter), kTfLiteOk);
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
