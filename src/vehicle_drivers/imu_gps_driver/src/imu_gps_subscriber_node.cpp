#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class IMUGPSSubscriber : public rclcpp::Node
{
public:
    IMUGPSSubscriber() : Node("imu_gps_subscriber_node")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", 10,
            std::bind(&IMUGPSSubscriber::imu_callback, this, std::placeholders::_1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/fix", 10,
            std::bind(&IMUGPSSubscriber::gps_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscriber node started");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "IMU: orient(w=%f,x=%f,y=%f,z=%f)  ang_vel(x=%f,y=%f,z=%f)  lin_acc(x=%f,y=%f,z=%f)",
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z,
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "GPS: lat=%f, lon=%f, alt=%f",
            msg->latitude, msg->longitude, msg->altitude);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUGPSSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}