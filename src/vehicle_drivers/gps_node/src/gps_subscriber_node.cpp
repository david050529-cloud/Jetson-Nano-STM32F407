#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// 定义一个GPS订阅节点类，继承自rclcpp::Node
class GpsSubscriberNode : public rclcpp::Node
{
public:
  // 构造函数，初始化节点名称为"gps_subscriber_node"
  GpsSubscriberNode() : Node("gps_subscriber_node")
  {
    // 创建一个订阅器，订阅"/gps/fix"主题，队列大小为10
    sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10,
        // 使用lambda函数作为回调函数，处理接收到的GPS数据
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
        {
          // 打印接收到的GPS数据，包括纬度、经度和海拔
          RCLCPP_INFO(this->get_logger(), "GPS Fix: lat=%.6f, lon=%.6f, alt=%.1f",
                      msg->latitude, msg->longitude, msg->altitude);
        });
  }

private:
  // 定义一个订阅器，用于接收GPS数据
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
  // 初始化rclcpp
  rclcpp::init(argc, argv);
  // 创建一个GpsSubscriberNode节点实例
  auto node = std::make_shared<GpsSubscriberNode>();
  // 开始运行节点，处理回调函数
  rclcpp::spin(node);
  // 关闭rclcpp
  rclcpp::shutdown();
  return 0;
}