#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GpsSubscriberNode : public rclcpp::Node
{
public:
  GpsSubscriberNode() : Node("gps_subscriber_node")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "gps/gps_data",
      10,
      std::bind(&GpsSubscriberNode::subscribe_gps_data, this, std::placeholders::_1)
    );
  }

private:
  void subscribe_gps_data(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received GPS data: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}