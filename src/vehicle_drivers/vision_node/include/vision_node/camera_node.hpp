#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>

namespace vision_node
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  void captureLoop();
  void publishCallback();

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::thread capture_thread_;
  std::atomic<bool> running_{false};

  std::mutex frame_mutex_;
  cv::Mat latest_frame_;

  std::vector<int> encode_params_;
};

}  // namespace vision_node
