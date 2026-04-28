#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial_driver/serial_driver.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <array>
#include <chrono>

class IMUDriverNode : public rclcpp::Node
{
public:
    IMUDriverNode() : Node("imu_publisher_node")
    {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

        // 参数声明
        this->declare_parameter<std::string>("port", "/dev/IMU");
        this->declare_parameter<int>("baudrate", 9600);
        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baudrate").as_int();

        try
        {
            io_context_ = std::make_shared<drivers::common::IoContext>();
            drivers::serial_driver::SerialPortConfig config(
                baud,
                drivers::serial_driver::FlowControl::NONE,
                drivers::serial_driver::Parity::NONE,
                drivers::serial_driver::StopBits::ONE);
            serial_port_ = std::make_shared<drivers::serial_driver::SerialPort>(
                *io_context_, port, config);
            serial_port_->open();
            RCLCPP_INFO(this->get_logger(), "IMU serial port opened on %s @ %d baud", port.c_str(), baud);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "IMU serial open failed: %s", e.what());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&IMUDriverNode::readIMUData, this));
    }

    ~IMUDriverNode()
    {
        if (serial_port_ && serial_port_->is_open())
            serial_port_->close();
    }

private:
    void readIMUData()
    {
        if (!serial_port_ || !serial_port_->is_open()) return;

        try
        {
            std::vector<uint8_t> buffer(1024);
            size_t bytes_read = serial_port_->receive(buffer);
            if (bytes_read > 0)
            {
                for (size_t i = 0; i < bytes_read; ++i)
                    if (processByte(buffer[i]))
                        publishIMUData();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "IMU read error: %s", e.what());
        }
    }

    bool processByte(uint8_t byte)
    {
        buffer_[key_] = byte;
        key_++;
        if (buffer_[0] != 0x55) { key_ = 0; return false; }
        if (key_ < 11) return false;
        if (validateChecksum(buffer_))
        {
            parseIMUData(buffer_);
            key_ = 0;
            return true;
        }
        key_ = 0;
        return false;
    }

    bool validateChecksum(const std::array<uint8_t, 11> &data)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < 10; ++i) sum += data[i];
        return (sum & 0xFF) == data[10];
    }

    void parseIMUData(const std::array<uint8_t, 11> &data)
    {
        if (data[1] == 0x51) // 加速度
        {
            acceleration_[0] = toShort(data[2], data[3]) / 32768.0 * 16 * 9.8;
            acceleration_[1] = toShort(data[4], data[5]) / 32768.0 * 16 * 9.8;
            acceleration_[2] = toShort(data[6], data[7]) / 32768.0 * 16 * 9.8;
        }
        else if (data[1] == 0x52) // 角速度
        {
            angular_velocity_[0] = toShort(data[2], data[3]) / 32768.0 * 2000 * M_PI / 180.0;
            angular_velocity_[1] = toShort(data[4], data[5]) / 32768.0 * 2000 * M_PI / 180.0;
            angular_velocity_[2] = toShort(data[6], data[7]) / 32768.0 * 2000 * M_PI / 180.0;
        }
        else if (data[1] == 0x53) // 欧拉角（用于四元数）
        {
            orientation_euler_[0] = toShort(data[2], data[3]) / 32768.0 * M_PI;   // roll
            orientation_euler_[1] = toShort(data[4], data[5]) / 32768.0 * M_PI;   // pitch
            orientation_euler_[2] = toShort(data[6], data[7]) / 32768.0 * M_PI;   // yaw
        }
    }

    void publishIMUData()
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = acceleration_[0];
        msg.linear_acceleration.y = acceleration_[1];
        msg.linear_acceleration.z = acceleration_[2];
        msg.angular_velocity.x = angular_velocity_[0];
        msg.angular_velocity.y = angular_velocity_[1];
        msg.angular_velocity.z = angular_velocity_[2];

        // 欧拉角转四元数
        auto q = eulerToQuaternion(orientation_euler_[0], orientation_euler_[1], orientation_euler_[2]);
        msg.orientation.x = q[0];
        msg.orientation.y = q[1];
        msg.orientation.z = q[2];
        msg.orientation.w = q[3];

        // 协方差（暂时未知）
        msg.linear_acceleration_covariance.fill(0.0);
        msg.angular_velocity_covariance.fill(0.0);
        msg.orientation_covariance.fill(0.0);

        imu_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published IMU data");
    }

    std::array<double,4> eulerToQuaternion(double roll, double pitch, double yaw)
    {
        double qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
        double qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
        double qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
        double qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
        return {qx, qy, qz, qw};
    }

    int16_t toShort(uint8_t low, uint8_t high) { return static_cast<int16_t>((high << 8) | low); }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<uint8_t, 11> buffer_{};
    size_t key_ = 0;
    std::array<double, 3> acceleration_{};
    std::array<double, 3> angular_velocity_{};
    std::array<double, 3> orientation_euler_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}