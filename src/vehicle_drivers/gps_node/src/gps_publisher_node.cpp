#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>

class GpsPublisherNode : public rclcpp::Node
{
public:
  GpsPublisherNode() : Node("gps_publisher_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("port", "/dev/GPS");
    this->declare_parameter<int>("baudrate", 9600);

    std::string port = this->get_parameter("port").as_string();
    int baud = this->get_parameter("baudrate").as_int();

    // 打开串口
    try
    {
      serial_.setPort(port);
      serial_.setBaudrate(baud);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
      serial_.setTimeout(timeout);
      serial_.open();
      RCLCPP_INFO(this->get_logger(), "GPS serial port %s opened at %d baud", port.c_str(), baud);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open GPS port: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // 创建发布者
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    status_pub_ = this->create_publisher<sensor_msgs::msg::NavSatStatus>("/gps/status", 10);

    // 定时器：每 50ms 读取一次串口
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&GpsPublisherNode::readAndPublish, this));
  }

private:
  void readAndPublish()
  {
    if (!serial_.isOpen())
      return;

    std::string line;
    try
    {
      line = serial_.readline(256, "\n");
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "GPS read error: %s", e.what());
      return;
    }

    if (line.empty())
      return;

    // 去除末尾换行符
    if (!line.empty() && line.back() == '\n')
      line.pop_back();
    if (line.empty())
      return;

    // 调试：打印原始 NMEA 语句（可选，正式运行可改为 RCLCPP_DEBUG）
    // RCLCPP_INFO(this->get_logger(), "NMEA: %s", line.c_str());

    // 匹配任何包含 "GGA" 的语句（支持 $GPGGA, $GNGGA, $BDGGA 等）
    if (line.find("GGA") != std::string::npos)
    {
      parseGGA(line);
    }
  }

  void parseGGA(const std::string &nmea)
  {
    // 示例: $GNGGA,130103.000,,,,,0,00,25.5,,,,,,*7A
    // 有效示例: $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    std::vector<std::string> fields;
    std::stringstream ss(nmea);
    std::string field;
    while (std::getline(ss, field, ','))
    {
      fields.push_back(field);
    }

    if (fields.size() < 10)
    {
      RCLCPP_WARN(this->get_logger(), "GGA line has insufficient fields: %zu", fields.size());
      return;
    }

    // 时间戳 (fields[1]) 可忽略
    // 纬度字段索引 2, 方向索引 3
    // 经度字段索引 4, 方向索引 5
    // 质量字段索引 6
    // 卫星数索引 7
    // 海拔字段索引 9

    double lat = 0.0, lon = 0.0, alt = 0.0;
    int fix_quality = 0;

    // 解析质量
    if (fields.size() > 6 && !fields[6].empty())
    {
      fix_quality = std::stoi(fields[6]);
    }

    // 解析纬度
    if (fields.size() > 3 && !fields[2].empty() && !fields[3].empty())
    {
      lat = convertNmeaCoordinate(fields[2], fields[3][0]);
    }

    // 解析经度
    if (fields.size() > 5 && !fields[4].empty() && !fields[5].empty())
    {
      lon = convertNmeaCoordinate(fields[4], fields[5][0]);
    }

    // 解析海拔
    if (fields.size() > 9 && !fields[9].empty())
    {
      alt = std::stod(fields[9]);
    }

    // 创建 NavSatFix 消息
    auto fix_msg = sensor_msgs::msg::NavSatFix();
    fix_msg.header.stamp = this->now();
    fix_msg.header.frame_id = "gps";
    fix_msg.latitude = lat;
    fix_msg.longitude = lon;
    fix_msg.altitude = alt;

    // 状态设置
    fix_msg.status.status = (fix_quality > 0) ? 
        sensor_msgs::msg::NavSatStatus::STATUS_FIX : 
        sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    // 简单协方差（近似）
    fix_msg.position_covariance[0] = 1.0;
    fix_msg.position_covariance[4] = 1.0;
    fix_msg.position_covariance[8] = 1.0;
    fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    fix_pub_->publish(fix_msg);
    RCLCPP_INFO(this->get_logger(), "Published GPS fix: lat=%.6f, lon=%.6f, alt=%.1f, fix_quality=%d",
                lat, lon, alt, fix_quality);
  }

  double convertNmeaCoordinate(const std::string &coord, char dir)
  {
    // 格式: ddmm.mmmm (纬度) 或 dddmm.mmmm (经度)
    if (coord.empty())
      return 0.0;

    size_t dotPos = coord.find('.');
    if (dotPos == std::string::npos)
      return 0.0;

    // 分钟部分从小数点前两位开始
    // 例如: 4807.038 -> 度=48, 分=07.038
    double minutes = std::stod(coord.substr(dotPos - 2));
    double degrees = std::stod(coord.substr(0, dotPos - 2));

    double decimal = degrees + minutes / 60.0;
    if (dir == 'S' || dir == 'W')
      decimal = -decimal;
    return decimal;
  }

  serial::Serial serial_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}