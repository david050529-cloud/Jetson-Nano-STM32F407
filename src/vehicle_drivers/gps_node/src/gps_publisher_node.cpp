#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GpsPublisherNode : public rclcpp::Node
{
public:
  GpsPublisherNode() : Node("gps_publisher_node")
  {
    this->declare_parameter<std::string>("port", "/dev/gps_usb");
    port_ = this->get_parameter("port").as_string();

    RCLCPP_INFO(this->get_logger(), "GPS Publisher Node started on port: %s", port_.c_str());

    publisher_ = this->create_publisher<std_msgs::msg::String>("gps/gps_data", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GpsPublisherNode::publish_gps_data, this));

    RCLCPP_INFO(this->get_logger(), "Publishing GPS data every second...");
  }     
private:
  void publish_gps_data()
  {
    auto message = std_msgs::msg::String();
    message.data = "GPS data from port: " + port_;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published GPS data: %s", message.data.c_str());
  }

  std::string port_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}