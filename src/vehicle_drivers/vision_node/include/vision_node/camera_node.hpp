#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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

  // ── Camera device ─────────────────────────────────────────────────
  cv::VideoCapture cap_;

  // ── Publishers ─────────────────────────────────────────────────────
  // raw:   sensor_msgs::msg::Image (BGR8)  for local processing nodes
  //         → YOLO / road_detector subscribe with zero decode overhead
  // compressed: sensor_msgs::msg::CompressedImage (JPEG) for remote viz
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          pub_raw_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── Capture thread ─────────────────────────────────────────────────
  std::thread        capture_thread_;
  std::atomic<bool>  running_{false};

  // ── Frame buffer (swap pattern: short lock, no deep copy) ─────────
  std::mutex frame_mutex_;
  cv::Mat    latest_frame_;

  // ── Configuration ──────────────────────────────────────────────────
  int  width_;
  int  height_;
  int  fps_;
  bool publish_compressed_;
  int  jpeg_quality_;
  int  compressed_skip_;   // publish compressed only every N raw frames

  // ── FPS monitoring ─────────────────────────────────────────────────
  int         frame_count_{0};
  int         compressed_count_{0};
  rclcpp::Time fps_last_log_;
};

}  // namespace vision_node
