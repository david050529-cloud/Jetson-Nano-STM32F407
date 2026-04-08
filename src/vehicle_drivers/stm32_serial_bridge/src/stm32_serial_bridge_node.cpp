// stm32_serial_bridge_node.cpp
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <cstring>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "stm32_serial_bridge/msg/motor_command.hpp"
#include "stm32_serial_bridge/msg/telemetry.hpp"

#include "serial/serial.h"

#pragma pack(push, 1)
// 发送给STM32的命令包结构（紧凑对齐）
struct CommandPacket
{
    float motor1_target_rps; // 电机1目标转速（转/秒）
    float motor2_target_rps; // 电机2目标转速
    uint16_t servo_angle;    // 舵机角度（单位：度或其他）
};

// 从STM32接收的遥测数据包原始格式（22字节：2字节帧头 + 20字节数据）
struct TelemetryPacketRaw
{
    uint8_t header[2];       // 帧头 0xAA, 0x55
    float roll;              // 横滚角
    float pitch;             // 俯仰角
    float yaw;               // 偏航角
    float motor1_actual_rps; // 电机1实际转速
    float motor2_actual_rps; // 电机2实际转速
    // 无帧尾（或帧尾被忽略）
};
#pragma pack(pop)

constexpr uint8_t FRAME_HEADER1 = 0xAA;
constexpr uint8_t FRAME_HEADER2 = 0x55;
constexpr size_t PACKET_SIZE = sizeof(TelemetryPacketRaw); // 22字节

class Stm32SerialBridge : public rclcpp::Node
{
public:
    Stm32SerialBridge() : Node("stm32_serial_bridge"), running_(true)
    {
        // 声明并获取串口参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("serial_timeout_ms", 100);

        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        int timeout_ms = this->get_parameter("serial_timeout_ms").as_int();

        try
        {
            serial_.setPort(port);
            serial_.setBaudrate(baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms);
            serial_.setTimeout(to);
            serial_.open();
            RCLCPP_INFO(this->get_logger(), "串口 %s 打开成功", port.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "打开串口 %s 失败: %s", port.c_str(), e.what());
            throw;
        }

        // 订阅电机控制指令话题
        motor_cmd_sub_ = this->create_subscription<stm32_serial_bridge::msg::MotorCommand>(
            "motor_commands", 10,
            std::bind(&Stm32SerialBridge::motorCommandCallback, this, std::placeholders::_1));

        // 发布遥测数据话题
        telemetry_pub_ = this->create_publisher<stm32_serial_bridge::msg::Telemetry>("telemetry", 10);

        // 启动串口读取线程
        read_thread_ = std::thread(&Stm32SerialBridge::readLoop, this);

        RCLCPP_INFO(this->get_logger(), "STM32 串桥节点已启动");
    }

    ~Stm32SerialBridge()
    {
        running_ = false;
        if (read_thread_.joinable())
        {
            read_thread_.join();
        }
        if (serial_.isOpen())
        {
            serial_.close();
        }
    }

private:
    void motorCommandCallback(const stm32_serial_bridge::msg::MotorCommand::SharedPtr msg)
    {
        /* 收到ROS指令后打包成CommandPacket并通过串口发送 */
        CommandPacket cmd;
        cmd.motor1_target_rps = msg->motor1_target_rps;
        cmd.motor2_target_rps = msg->motor2_target_rps;
        cmd.servo_angle = msg->servo_angle;

        std::vector<uint8_t> buffer(reinterpret_cast<uint8_t *>(&cmd),
                                    reinterpret_cast<uint8_t *>(&cmd) + sizeof(CommandPacket));
        try
        {
            serial_.write(buffer);
            RCLCPP_DEBUG(this->get_logger(), "发送指令: rps1=%.2f, rps2=%.2f, 舵机=%d",
                         cmd.motor1_target_rps, cmd.motor2_target_rps, cmd.servo_angle);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "发送指令失败: %s", e.what());
        }
    }

    void readLoop()
    {
        /* 持续从串口读取数据，寻找帧头并解析完整遥测包 */
        std::vector<uint8_t> buffer;

        while (running_ && rclcpp::ok())
        {
            if (!serial_.isOpen())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            try
            {
                size_t available = serial_.available();
                if (available > 0)
                {
                    std::string raw = serial_.read(available);
                    buffer.insert(buffer.end(), raw.begin(), raw.end());
                }

                // 在缓冲区中搜索帧头并提取完整帧（22字节）
                while (buffer.size() >= PACKET_SIZE)
                {
                    // 查找帧头 0xAA 0x55
                    auto it = buffer.begin();
                    bool found = false;
                    for (; it <= buffer.end() - PACKET_SIZE; ++it)
                    {
                        if (*it == FRAME_HEADER1 && *(it + 1) == FRAME_HEADER2)
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        // 保留最后可能的部分帧头（最多 PACKET_SIZE-1 字节），丢弃之前的无效数据
                        if (buffer.size() > PACKET_SIZE)
                        {
                            buffer.erase(buffer.begin(), buffer.end() - (PACKET_SIZE - 1));
                        }
                        break;
                    }

                    // 丢弃帧头之前的数据
                    buffer.erase(buffer.begin(), it);

                    // 确保有足够数据
                    if (buffer.size() < PACKET_SIZE)
                    {
                        break;
                    }

                    // 提取遥测数据包
                    TelemetryPacketRaw packet;
                    std::memcpy(&packet, buffer.data(), PACKET_SIZE);

                    // 发布 ROS 消息
                    auto msg = stm32_serial_bridge::msg::Telemetry();
                    msg.roll = packet.roll;
                    msg.pitch = packet.pitch;
                    msg.yaw = packet.yaw;
                    msg.motor1_actual_rps = packet.motor1_actual_rps;
                    msg.motor2_actual_rps = packet.motor2_actual_rps;
                    telemetry_pub_->publish(msg);

                    RCLCPP_DEBUG(this->get_logger(),
                                 "收到遥测: roll=%.2f, pitch=%.2f, yaw=%.2f, rps1=%.2f, rps2=%.2f",
                                 packet.roll, packet.pitch, packet.yaw,
                                 packet.motor1_actual_rps, packet.motor2_actual_rps);

                    // 移除已处理的帧
                    buffer.erase(buffer.begin(), buffer.begin() + PACKET_SIZE);
                }

                if (available == 0)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "串口读取错误: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    rclcpp::Subscription<stm32_serial_bridge::msg::MotorCommand>::SharedPtr motor_cmd_sub_;
    rclcpp::Publisher<stm32_serial_bridge::msg::Telemetry>::SharedPtr telemetry_pub_;
    serial::Serial serial_;
    std::thread read_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Stm32SerialBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}