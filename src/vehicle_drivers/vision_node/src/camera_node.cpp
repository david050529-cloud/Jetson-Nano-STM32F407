#include "vision_node/camera_node.hpp"

#include <stdexcept>
#include <sstream>

namespace vision_node
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options)
{
  // ── Declare parameters ────────────────────────────────────────────
  declare_parameter("camera_id",           0);
  declare_parameter("width",              640);
  declare_parameter("height",             480);
  declare_parameter("fps",                 30);
  declare_parameter("publish_compressed",  true);
  declare_parameter("jpeg_quality",        80);
  declare_parameter("compressed_skip",      1);   // 1 = every frame
  declare_parameter("fourcc",        "YUYV");     // YUYV / MJPG / empty=auto

  const int camera_id           = get_parameter("camera_id").as_int();
  width_                       = get_parameter("width").as_int();
  height_                      = get_parameter("height").as_int();
  fps_                         = get_parameter("fps").as_int();
  publish_compressed_          = get_parameter("publish_compressed").as_bool();
  jpeg_quality_                = get_parameter("jpeg_quality").as_int();
  compressed_skip_             = get_parameter("compressed_skip").as_int();
  const std::string fourcc_str = get_parameter("fourcc").as_string();

  if (compressed_skip_ < 1) compressed_skip_ = 1;

  // ── Compose FOURCC ─────────────────────────────────────────────────
  int fourcc = 0;
  if (!fourcc_str.empty()) {
    // pad short strings with spaces so cv::VideoWriter::fourcc works
    std::string padded = fourcc_str;
    padded.resize(4, ' ');
    fourcc = cv::VideoWriter::fourcc(padded[0], padded[1], padded[2], padded[3]);
  }

  // ── Open camera ────────────────────────────────────────────────────
  // V4L2 gives direct ioctl access → lower overhead than default backend
  cap_.open(camera_id, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    RCLCPP_WARN(get_logger(), "V4L2 failed for camera %d, trying default backend", camera_id);
    cap_.open(camera_id);
  }
  if (!cap_.isOpened()) {
    throw std::runtime_error("Cannot open camera " + std::to_string(camera_id));
  }

  // ── Configure capture format ───────────────────────────────────────
  // Strategy: try requested FOURCC; if unsupported, fall back to auto
  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS,          fps_);
  // Minimal buffer: cap_.read() always returns the freshest frame
  cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);

  bool format_ok = false;
  if (fourcc != 0) {
    format_ok = cap_.set(cv::CAP_PROP_FOURCC, fourcc);
    if (!format_ok) {
      RCLCPP_WARN(get_logger(), "Requested FOURCC '%s' rejected by camera, using auto",
                  fourcc_str.c_str());
    }
  }

  // Read back actual capture settings (camera may negotiate down)
  const double actual_w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  const double actual_h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
  const double actual_f = cap_.get(cv::CAP_PROP_FPS);
  const double actual_4cc = cap_.get(cv::CAP_PROP_FOURCC);

  std::ostringstream fourcc_name;
  fourcc_name << static_cast<char>(static_cast<int>(actual_4cc) & 0xFF)
              << static_cast<char>((static_cast<int>(actual_4cc) >> 8) & 0xFF)
              << static_cast<char>((static_cast<int>(actual_4cc) >> 16) & 0xFF)
              << static_cast<char>((static_cast<int>(actual_4cc) >> 24) & 0xFF);

  RCLCPP_INFO(get_logger(),
    "Camera %d opened: %.0f×%.0f @ %.1f fps  format=%s  V4L2 buffer=1",
    camera_id, actual_w, actual_h, actual_f, fourcc_name.str().c_str());

  // ── Publishers ─────────────────────────────────────────────────────
  // Raw BGR8 → local processing nodes (YOLO, road_detector) with zero decode cost
  pub_raw_ = create_publisher<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS());

  if (publish_compressed_) {
    pub_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/image_raw/compressed", rclcpp::QoS(10));
  }

  // ── Start threads ──────────────────────────────────────────────────
  running_ = true;
  capture_thread_ = std::thread(&CameraNode::captureLoop, this);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / fps_));
  timer_ = create_wall_timer(period, std::bind(&CameraNode::publishCallback, this));

  fps_last_log_ = now();

  RCLCPP_INFO(get_logger(), "Camera node ready — raw BGR8 + %s JPEG  quality=%d",
              publish_compressed_ ? "optional" : "no", jpeg_quality_);
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

  // ── Eager read: drain stale buffers ────────────────────────────────
  for (int i = 0; i < 3; ++i) {
    cap_.read(frame);
  }

  while (running_) {
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Frame read failed");
      continue;
    }

    // YUYV → BGR conversion (SIMD-accelerated via OpenCV).
    // If the camera delivers raw BGR already (rare), cvtColor is cheap.
    // MJPEG frames are decoded by OpenCV's libjpeg inside cap_.read().
    cv::Mat bgr;
    const int native_fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
    const char* pf = reinterpret_cast<const char*>(&native_fourcc);

    // ── Convert to BGR8 if needed ────────────────────────────────────
    if (memcmp(pf, "YUYV", 4) == 0 || memcmp(pf, "YVYU", 4) == 0 ||
        memcmp(pf, "UYVY", 4) == 0 || memcmp(pf, "Y42B", 4) == 0) {
      cv::cvtColor(frame, bgr,
        memcmp(pf, "UYVY", 4) == 0 ? cv::COLOR_YUV2BGR_UYVY :
                                      cv::COLOR_YUV2BGR_YUYV);
    } else if (memcmp(pf, "MJPG", 4) == 0) {
      // MJPEG — frame is already BGR after cap_.read() via libjpeg
      bgr = frame;
    } else {
      // Unknown format — assume BGR; if wrong, downstream nodes will see garbage
      bgr = frame;
    }

    // ── Swap into shared buffer (lock held briefly) ──────────────────
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      cv::swap(latest_frame_, bgr);
    }
  }
}

void CameraNode::publishCallback()
{
  // ── Swap frame out of the shared buffer ────────────────────────────
  cv::Mat frame;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) return;
    cv::swap(frame, latest_frame_);
  }

  const auto stamp = now();

  // ── Publish raw BGR8 Image (primary — zero-overhead for local nodes) ──
  sensor_msgs::msg::Image raw_msg;
  raw_msg.header.stamp    = stamp;
  raw_msg.header.frame_id = "camera_link";
  raw_msg.height          = frame.rows;
  raw_msg.width           = frame.cols;
  raw_msg.encoding        = "bgr8";
  raw_msg.is_bigendian    = false;
  raw_msg.step            = static_cast<uint32_t>(frame.step);
  raw_msg.data.assign(frame.datastart, frame.dataend);
  pub_raw_->publish(std::move(raw_msg));

  // ── Optional compressed JPEG (remote viewing / bandwidth-limited links) ──
  if (publish_compressed_ && pub_compressed_) {
    compressed_count_++;
    if ((compressed_count_ % compressed_skip_) == 0) {
      std::vector<uint8_t> buf;
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      if (cv::imencode(".jpg", frame, buf, params)) {
        sensor_msgs::msg::CompressedImage comp_msg;
        comp_msg.header.stamp = stamp;
        comp_msg.header.frame_id = "camera_link";
        comp_msg.format = "jpeg";
        comp_msg.data   = std::move(buf);
        pub_compressed_->publish(std::move(comp_msg));
      }
    }
  }

  // ── Periodic FPS log ───────────────────────────────────────────────
  frame_count_++;
  const auto now_t = now();
  const double elapsed = (now_t - fps_last_log_).seconds();
  if (elapsed >= 5.0) {
    const double actual_fps = frame_count_ / elapsed;
    RCLCPP_INFO(get_logger(), "Camera FPS: %.1f  (target %d)  raw topic + %s",
                actual_fps, fps_,
                (publish_compressed_ && pub_compressed_) ? "compressed" : "raw-only");
    frame_count_ = 0;
    fps_last_log_ = now_t;
  }
}

}  // namespace vision_node

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vision_node::CameraNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_node"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
