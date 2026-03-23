#ifndef WHISTLE_DETECTOR_TFLITE_EXECUTER_HPP
#define WHISTLE_DETECTOR_TFLITE_EXECUTER_HPP

#include <cstdint>
#include <string>
#include <vector>

struct TfLiteInterpreter;
struct TfLiteDelegate;

namespace whistle_detector
{

class TFLiteExecuter
{
public:
    TFLiteExecuter() = default;
    ~TFLiteExecuter();

    TFLiteExecuter(TFLiteExecuter &) = delete;
    TFLiteExecuter(TFLiteExecuter &&) = delete;
    TFLiteExecuter &operator=(const TFLiteExecuter &) = delete;
    TFLiteExecuter &operator=(TFLiteExecuter &&) = delete;

    void loadModelFromFile(const std::string &file, const std::vector<int> &input_dims, int num_threads = 1);
    void loadModelFromArray(const void *model_data, size_t length, const std::vector<int> &input_dims, int num_threads = 1);

    float *getInputTensor();
    const float *getOutputTensor() const;

    size_t getElementsInputTensor() const;
    size_t getElementsOutputTensor() const;

    void execute();

private:
    TfLiteInterpreter *interpreter = nullptr;
    TfLiteDelegate *delegate = nullptr;
};

}  // namespace whistle_detector

#endif
