#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GpsSubscriberNode : public rclcpp::Node
{
public:
  GpsSubscriberNode() : Node("gps_subscriber_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
        {
          RCLCPP_INFO(this->get_logger(), "GPS Fix: lat=%.6f, lon=%.6f, alt=%.1f",
                      msg->latitude, msg->longitude, msg->altitude);
        });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}