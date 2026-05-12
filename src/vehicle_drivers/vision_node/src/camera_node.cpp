#include "vision_node/camera_node.hpp"

#include <stdexcept>
#include <sstream>

namespace vision_node
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options)
{
  declare_parameter("camera_id",           0);
  declare_parameter("width",              640);
  declare_parameter("height",             480);
  declare_parameter("fps",                 30);
  declare_parameter("publish_compressed",  true);
  declare_parameter("jpeg_quality",        80);
  declare_parameter("compressed_skip",      1);
  declare_parameter("fourcc",        "MJPG");   // camera-side encode, no Jetson hw needed

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
    std::string padded = fourcc_str;
    padded.resize(4, ' ');
    fourcc = cv::VideoWriter::fourcc(padded[0], padded[1], padded[2], padded[3]);
  }

  // ── Open camera ────────────────────────────────────────────────────
  cap_.open(camera_id, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    RCLCPP_WARN(get_logger(), "V4L2 failed for camera %d, trying default backend", camera_id);
    cap_.open(camera_id);
  }
  if (!cap_.isOpened()) {
    throw std::runtime_error("Cannot open camera " + std::to_string(camera_id));
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS,          fps_);
  cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);

  if (fourcc != 0) {
    bool ok = cap_.set(cv::CAP_PROP_FOURCC, fourcc);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "FOURCC '%s' rejected, using camera default", fourcc_str.c_str());
    }
  }

  const double actual_w   = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  const double actual_h   = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
  const double actual_f   = cap_.get(cv::CAP_PROP_FPS);
  const double actual_4cc = cap_.get(cv::CAP_PROP_FOURCC);

  std::ostringstream fourcc_name;
  fourcc_name << static_cast<char>(static_cast<int>(actual_4cc) & 0xFF)
              << static_cast<char>((static_cast<int>(actual_4cc) >> 8) & 0xFF)
              << static_cast<char>((static_cast<int>(actual_4cc) >> 16) & 0xFF)
              << static_cast<char>((static_cast<int>(actual_4cc) >> 24) & 0xFF);

  RCLCPP_INFO(get_logger(),
    "Camera %d: %.0fx%.0f @ %.1ffps  format=%s  buffer=1",
    camera_id, actual_w, actual_h, actual_f, fourcc_name.str().c_str());

  // ── Publishers ─────────────────────────────────────────────────────
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

  RCLCPP_INFO(get_logger(),
    "Camera node ready — hot path: capture→convert→encode in cap-thread, "
    "publish in timer. JPEG encode %s (quality=%d)",
    publish_compressed_ ? "ON" : "OFF", jpeg_quality_);
}

CameraNode::~CameraNode()
{
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  cap_.release();
}

// ═══════════════════════════════════════════════════════════════════════
// Capture thread: runs at camera frame rate.
// Does ALL heavy lifting: YUV→BGR + optional JPEG encode.
// Only swaps FramePacket into shared buffer at the end.
// ═══════════════════════════════════════════════════════════════════════
void CameraNode::captureLoop()
{
  cv::Mat frame;

  // Determine YUV→BGR conversion code from actual FOURCC
  const int native_fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
  int yuv2bgr_code = cv::COLOR_YUV2BGR_YUYV;
  {
    const char* pf = reinterpret_cast<const char*>(&native_fourcc);
    if (memcmp(pf, "UYVY", 4) == 0) yuv2bgr_code = cv::COLOR_YUV2BGR_UYVY;
  }
  RCLCPP_INFO(get_logger(), "Capture thread started, FOURCC=0x%08x", native_fourcc);

  // JPEG encode parameters (pre-allocated)
  const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

  // Drain stale V4L2 buffers
  for (int i = 0; i < 3; ++i) cap_.read(frame);

  int encode_seq = 0;  // for compressed_skip counting

  while (running_) {
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Frame read failed");
      continue;
    }

    // ── Step 1: YUV→BGR conversion ───────────────────────────────────
    cv::Mat bgr;
    const int channels = frame.channels();
    if (channels == 2) {
      cv::cvtColor(frame, bgr, yuv2bgr_code);
    } else if (channels == 1) {
      cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    } else {
      bgr = frame;
    }

    // ── Step 2: optional JPEG encoding (the expensive part) ──────────
    std::vector<uint8_t> jpeg;
    if (publish_compressed_) {
      encode_seq++;
      if ((encode_seq % compressed_skip_) == 0) {
        cv::imencode(".jpg", bgr, jpeg, jpeg_params);
      }
    }

    // ── Step 3: swap pre-encoded packet into shared buffer ───────────
    {
      FramePacket pkt;
      cv::swap(pkt.bgr, bgr);
      pkt.jpeg = std::move(jpeg);
      pkt.stamp = now();

      std::lock_guard<std::mutex> lock(frame_mutex_);
      std::swap(latest_packet_, pkt);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════
// Timer callback: runs at target FPS.
// Only does lightweight work — swap out pre-encoded packet, publish.
// JPEG encoding was already done by the capture thread.
// ═══════════════════════════════════════════════════════════════════════
void CameraNode::publishCallback()
{
  FramePacket pkt;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_packet_.bgr.empty()) return;
    pkt = std::move(latest_packet_);
    // latest_packet_ is now empty — capture thread will fill a fresh one
  }

  // ── Publish raw BGR8 Image ─────────────────────────────────────────
  sensor_msgs::msg::Image raw_msg;
  raw_msg.header.stamp    = pkt.stamp;
  raw_msg.header.frame_id = "camera_link";
  raw_msg.height          = pkt.bgr.rows;
  raw_msg.width           = pkt.bgr.cols;
  raw_msg.encoding        = "bgr8";
  raw_msg.is_bigendian    = false;
  raw_msg.step            = static_cast<uint32_t>(pkt.bgr.step);
  raw_msg.data.assign(pkt.bgr.datastart, pkt.bgr.dataend);
  pub_raw_->publish(std::move(raw_msg));

  // ── Publish compressed JPEG (already encoded by capture thread) ────
  if (publish_compressed_ && pub_compressed_ && !pkt.jpeg.empty()) {
    sensor_msgs::msg::CompressedImage comp_msg;
    comp_msg.header.stamp    = pkt.stamp;
    comp_msg.header.frame_id = "camera_link";
    comp_msg.format          = "jpeg";
    comp_msg.data            = std::move(pkt.jpeg);
    pub_compressed_->publish(std::move(comp_msg));
  }

  // ── Periodic FPS log ───────────────────────────────────────────────
  frame_count_++;
  const auto now_t = now();
  const double elapsed = (now_t - fps_last_log_).seconds();
  if (elapsed >= 5.0) {
    RCLCPP_INFO(get_logger(), "Publish FPS: %.1f  (target %d)  %s",
                frame_count_ / elapsed, fps_,
                publish_compressed_ ? "+JPEG" : "raw-only");
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
