#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
        : Node("camera_node"), transport_initialized_(false)
    {
        this->declare_parameter("camera_id", 0);
        int camera_id = this->get_parameter("camera_id").as_int();

        cap_.open(camera_id);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera %d", camera_id);
            rclcpp::shutdown();
        }

        // 定时器：30ms 一帧（约33fps）
        timer_ = this->create_wall_timer(30ms, std::bind(&CameraNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 延迟初始化 image_transport_ 和 publisher_
        if (!transport_initialized_) {
            try {
                image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
                // 获取 image_transport 参数（launch 中可设为 h264）
                std::string transport = this->declare_parameter<std::string>("image_transport", "raw");
                pub_ = image_transport_->advertise("/camera/image_raw", 10);
                transport_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "ImageTransport initialized with transport: %s", transport.c_str());
            } catch (const std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Failed to init ImageTransport: %s", e.what());
                rclcpp::shutdown();
                return;
            }
        }

        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        // 发布原始 bgr8 图像，image_transport 会在传输侧用 x264enc 压缩
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        pub_.publish(*msg);
    }

    cv::VideoCapture cap_;
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool transport_initialized_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}