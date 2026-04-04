#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial_driver/serial_driver.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

class IMUDriverNode : public rclcpp::Node
{
public:
    IMUDriverNode() : Node("imu_driver_node")
    {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

        try
        {
            // 创建IO上下文
            io_context_ = std::make_shared<drivers::common::IoContext>();

            // 配置串口参数 - 使用正确的参数类型
            // SerialPortConfig(uint32_t baud_rate, FlowControl flow_control, Parity parity, StopBits stop_bits)
            drivers::serial_driver::SerialPortConfig config(
                9600, // 波特率直接使用数值
                drivers::serial_driver::FlowControl::NONE,
                drivers::serial_driver::Parity::NONE,
                drivers::serial_driver::StopBits::ONE);

            // 创建串口对象（需要提供IO上下文、端口名和配置）
            std::string port_name = "/dev/imu_usb";
            serial_port_ = std::make_shared<drivers::serial_driver::SerialPort>(
                *io_context_,
                port_name,
                config);

            // 打开串口
            serial_port_->open();
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully on %s with baudrate 9600", port_name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&IMUDriverNode::readIMUData, this));
    }

    ~IMUDriverNode()
    {
        if (serial_port_ && serial_port_->is_open())
        {
            try
            {
                serial_port_->close();
                RCLCPP_INFO(this->get_logger(), "Serial port closed.");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error closing serial port: %s", e.what());
            }
        }
    }

private:
    void readIMUData()
    {
        if (!serial_port_ || !serial_port_->is_open())
        {
            return;
        }

        try
        {
            // receive只接受一个参数，返回读取的字节数
            std::vector<uint8_t> buffer(1024);
            size_t bytes_read = serial_port_->receive(buffer);

            if (bytes_read > 0)
            {
                for (size_t i = 0; i < bytes_read; ++i)
                {
                    if (processByte(buffer[i]))
                    {
                        publishIMUData();
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
        }
    }

    bool processByte(uint8_t byte)
    {
        buffer_[key_] = byte;
        key_++;

        // 检查帧头
        if (buffer_[0] != 0x55)
        {
            key_ = 0;
            return false;
        }

        // 等待接收完整的一帧数据（11字节）
        if (key_ < 11)
        {
            return false;
        }

        // 验证校验和
        if (validateChecksum(buffer_))
        {
            parseIMUData(buffer_);
            key_ = 0;
            return true;
        }
        else
        {
            key_ = 0;
            return false;
        }
    }

    bool validateChecksum(const std::array<uint8_t, 11> &data)
    {
        uint8_t checksum = 0;
        for (size_t i = 0; i < 10; ++i)
        {
            checksum += data[i];
        }
        return (checksum & 0xFF) == data[10];
    }

    void parseIMUData(const std::array<uint8_t, 11> &data)
    {
        if (data[1] == 0x51) // 加速度数据
        {
            acceleration_[0] = toShort(data[2], data[3]) / 32768.0 * 16 * 9.8;
            acceleration_[1] = toShort(data[4], data[5]) / 32768.0 * 16 * 9.8;
            acceleration_[2] = toShort(data[6], data[7]) / 32768.0 * 16 * 9.8;

            RCLCPP_DEBUG(this->get_logger(), "Acc: %.2f, %.2f, %.2f",
                         acceleration_[0], acceleration_[1], acceleration_[2]);
        }
        else if (data[1] == 0x52) // 角速度数据
        {
            angular_velocity_[0] = toShort(data[2], data[3]) / 32768.0 * 2000 * M_PI / 180.0;
            angular_velocity_[1] = toShort(data[4], data[5]) / 32768.0 * 2000 * M_PI / 180.0;
            angular_velocity_[2] = toShort(data[6], data[7]) / 32768.0 * 2000 * M_PI / 180.0;

            RCLCPP_DEBUG(this->get_logger(), "Gyro: %.2f, %.2f, %.2f",
                         angular_velocity_[0], angular_velocity_[1], angular_velocity_[2]);
        }
        else if (data[1] == 0x53) // 角度数据
        {
            orientation_[0] = toShort(data[2], data[3]) / 32768.0 * M_PI;
            orientation_[1] = toShort(data[4], data[5]) / 32768.0 * M_PI;
            orientation_[2] = toShort(data[6], data[7]) / 32768.0 * M_PI;

            RCLCPP_DEBUG(this->get_logger(), "Angle: %.2f, %.2f, %.2f",
                         orientation_[0] * 180.0 / M_PI, orientation_[1] * 180.0 / M_PI, orientation_[2] * 180.0 / M_PI);
        }
    }

    void publishIMUData()
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        // 设置加速度数据
        imu_msg.linear_acceleration.x = acceleration_[0];
        imu_msg.linear_acceleration.y = acceleration_[1];
        imu_msg.linear_acceleration.z = acceleration_[2];

        // 设置角速度数据
        imu_msg.angular_velocity.x = angular_velocity_[0];
        imu_msg.angular_velocity.y = angular_velocity_[1];
        imu_msg.angular_velocity.z = angular_velocity_[2];

        // 将欧拉角转换为四元数
        auto quaternion = eulerToQuaternion(orientation_[0], orientation_[1], orientation_[2]);
        imu_msg.orientation.x = quaternion[0];
        imu_msg.orientation.y = quaternion[1];
        imu_msg.orientation.z = quaternion[2];
        imu_msg.orientation.w = quaternion[3];

        // 设置协方差矩阵（如果不知道，设置为-1表示未知）
        imu_msg.linear_acceleration_covariance.fill(0.0);
        imu_msg.angular_velocity_covariance.fill(0.0);
        imu_msg.orientation_covariance.fill(0.0);

        imu_publisher_->publish(imu_msg);

        RCLCPP_INFO(this->get_logger(), "Published IMU data");
    }

    std::array<double, 4> eulerToQuaternion(double roll, double pitch, double yaw)
    {
        double qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        double qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        double qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
        double qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        return {qx, qy, qz, qw};
    }

    int16_t toShort(uint8_t low, uint8_t high)
    {
        return static_cast<int16_t>((high << 8) | low);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<uint8_t, 11> buffer_{};
    size_t key_ = 0;

    std::array<double, 3> acceleration_{};
    std::array<double, 3> angular_velocity_{};
    std::array<double, 3> orientation_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}