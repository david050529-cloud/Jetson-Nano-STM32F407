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
        : Node("camera_node")
    {
        // 声明摄像头参数
        this->declare_parameter("camera_id", 0);
        int camera_id = this->get_parameter("camera_id").as_int();

        cap_.open(camera_id);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera %d", camera_id);
            rclcpp::shutdown();
        }

        // 使用 image_transport 发布原始图像，实际编码由 transport 插件处理（如 h264）
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        pub_ = image_transport_->advertise("/camera/image_raw", 10);

        // 定时器：30ms 一帧 (~33fps)
        timer_ = this->create_wall_timer(30ms, std::bind(&CameraNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        // 发布原始 bgr8 图像，让 image_transport 在传输侧压缩
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        pub_.publish(*msg);
    }

    cv::VideoCapture cap_;
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}