#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // 声明参数：摄像头设备号（默认0）
        this->declare_parameter("camera_id", 0);
        int camera_id = this->get_parameter("camera_id").as_int();

        // 初始化摄像头
        cap_.open(camera_id);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera %d", camera_id);
            rclcpp::shutdown();
        }

        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

        // 定时器：每30ms发布一帧（约33帧/秒）
        timer_ = this->create_wall_timer(30ms, std::bind(&CameraNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        // 使用cv_bridge将OpenCV图像转换为ROS图像消息
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}