#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMUClientNode : public rclcpp::Node
{
public:
    IMUClientNode() : Node("imu_client_node")
    {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", 10, std::bind(&IMUClientNode::imuCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received IMU Data:");
        RCLCPP_INFO(this->get_logger(), "Linear Acceleration: [x: %f, y: %f, z: %f]",
                    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        RCLCPP_INFO(this->get_logger(), "Angular Velocity: [x: %f, y: %f, z: %f]",
                    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: [x: %f, y: %f, z: %f, w: %f]",
                    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}