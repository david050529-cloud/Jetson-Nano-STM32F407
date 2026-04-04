#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "stm32_serial_bridge/msg/motor_servo_cmd.hpp"
#include "stm32_serial_bridge/msg/telemetry.hpp"
#include <serial/serial.h>

// 与下位机完全一致的结构体定义（必须packed）
#pragma pack(push, 1)
struct CommandPacket
{
    float motor1_target_rps;
    float motor2_target_rps;
    uint16_t servo_angle;
};

struct TelemetryPacket
{
    float roll;
    float pitch;
    float yaw;
    float motor1_actual_rps;
    float motor2_actual_rps;
};
#pragma pack(pop)

class BalanceBotCommNode : public rclcpp::Node
{
public:
    BalanceBotCommNode()
        : Node("balance_bot_comm_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("timeout_ms", 100);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        int timeout_ms = this->get_parameter("timeout_ms").as_int();

        // 初始化串口
        try
        {
            serial_.setPort(port);
            serial_.setBaudrate(baudrate);
            serial::Timeout timeout{serial::Timeout::simpleTimeout(timeout_ms)};
            serial_.setTimeout(timeout);
            serial_.open();
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened", port.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 订阅指令主题
        cmd_sub_ = this->create_subscription<stm32_serial_bridge::msg::MotorServoCmd>(
            "motor_servo_cmd", 10,
            std::bind(&BalanceBotCommNode::cmdCallback, this, std::placeholders::_1));

        // 发布遥测主题
        telemetry_pub_ = this->create_publisher<stm32_serial_bridge::msg::Telemetry>("telemetry", 10);

        // 启动接收线程
        recv_thread_ = std::thread(&BalanceBotCommNode::receiveLoop, this);
    }

    ~BalanceBotCommNode()
    {
        running_ = false;
        if (recv_thread_.joinable())
            recv_thread_.join();
        if (serial_.isOpen())
            serial_.close();
    }

private:
    void cmdCallback(const stm32_serial_bridge::msg::MotorServoCmd::SharedPtr msg)
    {
        CommandPacket packet;
        packet.motor1_target_rps = msg->motor1_target_rps;
        packet.motor2_target_rps = msg->motor2_target_rps;
        packet.servo_angle = msg->servo_angle;

        // 直接发送二进制数据
        serial_.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    }

    void receiveLoop()
    {
        uint8_t buffer[sizeof(TelemetryPacket)];
        while (running_ && rclcpp::ok())
        {
            // 阻塞读取一帧数据（大小固定）
            size_t len = serial_.read(buffer, sizeof(TelemetryPacket));
            if (len == sizeof(TelemetryPacket))
            {
                TelemetryPacket raw;
                std::memcpy(&raw, buffer, sizeof(raw));

                auto msg = stm32_serial_bridge::msg::Telemetry();
                msg.roll = raw.roll;
                msg.pitch = raw.pitch;
                msg.yaw = raw.yaw;
                msg.motor1_actual_rps = raw.motor1_actual_rps;
                msg.motor2_actual_rps = raw.motor2_actual_rps;

                telemetry_pub_->publish(msg);
            }
            else if (len > 0)
            {
                RCLCPP_WARN(this->get_logger(), "Received incomplete frame (%zu bytes)", len);
            }
            // 短暂休眠防止CPU空转（可去掉，因为read会阻塞）
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    serial::Serial serial_;
    std::thread recv_thread_;
    std::atomic<bool> running_{true};

    rclcpp::Subscription<stm32_serial_bridge::msg::MotorServoCmd>::SharedPtr cmd_sub_;
    rclcpp::Publisher<stm32_serial_bridge::msg::Telemetry>::SharedPtr telemetry_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalanceBotCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}