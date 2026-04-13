#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <atomic>
#include <memory>
#include <thread>

#include "whistle_detector/alsa_whistle_detector.hpp"

namespace whistle_detector
{

namespace
{
constexpr double kDefaultPublishConfidenceThreshold = 0.998600;

int defaultNumThreads()
{
    const unsigned int concurrency = std::thread::hardware_concurrency();
    return static_cast<int>(std::max(1u, std::min(2u, concurrency == 0 ? 1u : concurrency)));
}
}

class WhistleDetectorNode : public rclcpp::Node
{
public:
    WhistleDetectorNode() : rclcpp::Node("whistle_detector_node")
    {
        declare_parameter<std::string>("device_name", "default");
        declare_parameter<std::string>("publish_topic", "/whistle_detection/detected");
        declare_parameter<double>("cooldown_ms", 1500.0);
        declare_parameter<int>("num_threads", defaultNumThreads());
        declare_parameter<double>("publish_confidence_threshold", kDefaultPublishConfidenceThreshold);

        const auto share_dir = ament_index_cpp::get_package_share_directory("whistle_detector");
        const auto model_dir = share_dir + "/data";

        detector_ = std::make_unique<AlsaWhistleDetector>(
            model_dir,
            get_parameter("num_threads").as_int());
        publisher_ = create_publisher<std_msgs::msg::Bool>(
            get_parameter("publish_topic").as_string(), 10);

        detector_->setDetectionCallback([this](float confidence) {
            std_msgs::msg::Bool msg;
            msg.data = true;
            publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "whistle detected with confidence %.6f", confidence);
        });

        worker_ = std::thread([this]() {
            try {
                detector_->run(
                    get_parameter("device_name").as_string(),
                    std::chrono::milliseconds(static_cast<int64_t>(get_parameter("cooldown_ms").as_double())),
                    static_cast<float>(get_parameter("publish_confidence_threshold").as_double()));
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "whistle detector stopped: %s", e.what());
            }
        });
    }

    ~WhistleDetectorNode() override
    {
        if (detector_ != nullptr) {
            detector_->requestStop();
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    std::unique_ptr<AlsaWhistleDetector> detector_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    std::thread worker_;
};

}  // namespace whistle_detector

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<whistle_detector::WhistleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
