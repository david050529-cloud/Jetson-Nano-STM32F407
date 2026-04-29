#include "vision_node/camera_node.hpp"

#include <stdexcept>

namespace vision_node
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options)
{
  declare_parameter("camera_id",    0);
  declare_parameter("width",        640);
  declare_parameter("height",       480);
  declare_parameter("fps",          30);
  declare_parameter("jpeg_quality", 80);

  const int camera_id    = get_parameter("camera_id").as_int();
  const int width        = get_parameter("width").as_int();
  const int height       = get_parameter("height").as_int();
  const int fps          = get_parameter("fps").as_int();
  const int jpeg_quality = get_parameter("jpeg_quality").as_int();

  encode_params_ = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};

  // V4L2 backend: direct hardware access, lower overhead than default backend
  cap_.open(camera_id, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    RCLCPP_WARN(get_logger(), "V4L2 backend unavailable, retrying with default backend");
    cap_.open(camera_id);
  }
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Cannot open camera %d", camera_id);
    throw std::runtime_error("Camera open failed");
  }

  // MJPEG: camera encodes JPEG at hardware level, drastically reduces USB bandwidth
  cap_.set(cv::CAP_PROP_FOURCC,      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap_.set(cv::CAP_PROP_FPS,          fps);
  // Buffer of 1 so cap_.read() always returns the newest frame
  cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);

  // BEST_EFFORT + KeepLast(1): drop stale frames instead of queuing them
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    "/camera/image_raw/compressed", qos);

  running_ = true;
  capture_thread_ = std::thread(&CameraNode::captureLoop, this);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / fps));
  timer_ = create_wall_timer(period, std::bind(&CameraNode::publishCallback, this));

  RCLCPP_INFO(get_logger(), "Camera node started: %dx%d @ %d fps  quality=%d",
    width, height, fps, jpeg_quality);
}

CameraNode::~CameraNode()
{
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  cap_.release();
}

void CameraNode::captureLoop()
{
  cv::Mat frame;
  while (running_) {
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Frame read failed");
      continue;
    }
    // swap avoids memcopy: latest_frame_ gets new data, frame reuses old buffer
    std::lock_guard<std::mutex> lock(frame_mutex_);
    cv::swap(latest_frame_, frame);
  }
}

void CameraNode::publishCallback()
{
  cv::Mat frame;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) {
      return;
    }
    // Swap out to minimise lock duration; encoding happens outside the lock
    cv::swap(frame, latest_frame_);
  }

  std::vector<uint8_t> buf;
  if (!cv::imencode(".jpg", frame, buf, encode_params_)) {
    RCLCPP_WARN(get_logger(), "JPEG encode failed");
    return;
  }

  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = now();
  msg.format       = "jpeg";
  msg.data         = std::move(buf);  // zero-copy move into message
  pub_->publish(std::move(msg));
}

}  // namespace vision_node

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision_node::CameraNode>());
  rclcpp::shutdown();
  return 0;
}
