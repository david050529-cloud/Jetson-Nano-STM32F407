#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <vector>
#include <chrono>

class IMUGPSParser : public rclcpp::Node
{
public:
    IMUGPSParser() : Node("imu_gps_parser_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 9600);
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baudrate").as_int();

        // 创建发布者
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);

        // 打开串口
        if (!open_serial(port, baud)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        // 创建定时器，定期读取串口数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&IMUGPSParser::read_serial_callback, this));
    }

    ~IMUGPSParser()
    {
        if (serial_fd_ >= 0) close(serial_fd_);
    }

private:
    // 串口操作
    int serial_fd_ = -1;
    bool open_serial(const std::string& port, int baudrate)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;

        struct termios options;
        tcgetattr(serial_fd_, &options);
        cfsetispeed(&options, get_baud_rate(baudrate));
        cfsetospeed(&options, get_baud_rate(baudrate));
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;
        tcsetattr(serial_fd_, TCSANOW, &options);
        tcflush(serial_fd_, TCIOFLUSH);
        return true;
    }

    speed_t get_baud_rate(int baudrate)
    {
        switch(baudrate) {
            case 4800: return B4800;
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default: return B9600;
        }
    }

    // 数据缓冲
    std::vector<uint8_t> buffer_;

    // 解析得到的临时数据
    struct ImuData {
        double ax = 0.0, ay = 0.0, az = 0.0;   // 加速度 (m/s^2)
        double gx = 0.0, gy = 0.0, gz = 0.0;   // 角速度 (rad/s)
        double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // 四元数
        bool has_acc = false;
        bool has_gyro = false;
        bool has_quat = false;
    } imu_data_;

    struct GpsData {
        double lat = 0.0, lon = 0.0;
        bool has_data = false;
    } gps_data_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 校验和计算
    uint8_t calc_checksum(const std::vector<uint8_t>& packet)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < packet.size() - 1; ++i) sum += packet[i];
        return sum;
    }

    // 解析加速度包 (0x51)
    void parse_accel(const std::vector<uint8_t>& packet)
    {
        if (packet.size() < 11) return;
        int16_t ax_raw = (int16_t)(packet[3] << 8 | packet[2]);
        int16_t ay_raw = (int16_t)(packet[5] << 8 | packet[4]);
        int16_t az_raw = (int16_t)(packet[7] << 8 | packet[6]);
        // 转换: 1 LSB = 16g / 32768 = 0.00048828125 g, g = 9.80665 m/s^2
        const double scale = 16.0 / 32768.0 * 9.80665;
        imu_data_.ax = ax_raw * scale;
        imu_data_.ay = ay_raw * scale;
        imu_data_.az = az_raw * scale;
        imu_data_.has_acc = true;
    }

    // 解析角速度包 (0x52)
    void parse_gyro(const std::vector<uint8_t>& packet)
    {
        if (packet.size() < 11) return;
        int16_t gx_raw = (int16_t)(packet[3] << 8 | packet[2]);
        int16_t gy_raw = (int16_t)(packet[5] << 8 | packet[4]);
        int16_t gz_raw = (int16_t)(packet[7] << 8 | packet[6]);
        // 转换: ±2000°/s -> 2000/32768 = 0.06103515625 °/s per LSB, 转 rad/s
        const double scale = 2000.0 / 32768.0 * M_PI / 180.0;
        imu_data_.gx = gx_raw * scale;
        imu_data_.gy = gy_raw * scale;
        imu_data_.gz = gz_raw * scale;
        imu_data_.has_gyro = true;
    }

    // 解析四元数包 (0x59)
    void parse_quaternion(const std::vector<uint8_t>& packet)
    {
        if (packet.size() < 11) return;
        int16_t q0_raw = (int16_t)(packet[3] << 8 | packet[2]);
        int16_t q1_raw = (int16_t)(packet[5] << 8 | packet[4]);
        int16_t q2_raw = (int16_t)(packet[7] << 8 | packet[6]);
        int16_t q3_raw = (int16_t)(packet[9] << 8 | packet[8]);
        const double scale = 1.0 / 32768.0;
        imu_data_.q0 = q0_raw * scale;
        imu_data_.q1 = q1_raw * scale;
        imu_data_.q2 = q2_raw * scale;
        imu_data_.q3 = q3_raw * scale;
        imu_data_.has_quat = true;
    }

    // 解析经纬度包 (0x57)
    void parse_gps(const std::vector<uint8_t>& packet)
    {
        if (packet.size() < 11) return;
        uint32_t lon_raw = (uint32_t)(packet[5] << 24 | packet[4] << 16 | packet[3] << 8 | packet[2]);
        uint32_t lat_raw = (uint32_t)(packet[9] << 24 | packet[8] << 16 | packet[7] << 8 | packet[6]);

        // 转换为度
        double lon_deg = lon_raw / 10000000.0;
        double lon_min = (lon_raw % 10000000) / 100000.0;
        double lat_deg = lat_raw / 10000000.0;
        double lat_min = (lat_raw % 10000000) / 100000.0;
        gps_data_.lon = lon_deg + lon_min / 60.0;
        gps_data_.lat = lat_deg + lat_min / 60.0;
        gps_data_.has_data = true;
    }

    // 发布IMU消息
    void publish_imu()
    {
        if (!imu_data_.has_acc && !imu_data_.has_gyro && !imu_data_.has_quat) return;
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        if (imu_data_.has_quat) {
            msg.orientation.w = imu_data_.q0;
            msg.orientation.x = imu_data_.q1;
            msg.orientation.y = imu_data_.q2;
            msg.orientation.z = imu_data_.q3;
            msg.orientation_covariance[0] = 0.001;
            msg.orientation_covariance[4] = 0.001;
            msg.orientation_covariance[8] = 0.001;
        }
        if (imu_data_.has_gyro) {
            msg.angular_velocity.x = imu_data_.gx;
            msg.angular_velocity.y = imu_data_.gy;
            msg.angular_velocity.z = imu_data_.gz;
            msg.angular_velocity_covariance[0] = 0.001;
            msg.angular_velocity_covariance[4] = 0.001;
            msg.angular_velocity_covariance[8] = 0.001;
        }
        if (imu_data_.has_acc) {
            msg.linear_acceleration.x = imu_data_.ax;
            msg.linear_acceleration.y = imu_data_.ay;
            msg.linear_acceleration.z = imu_data_.az;
            msg.linear_acceleration_covariance[0] = 0.01;
            msg.linear_acceleration_covariance[4] = 0.01;
            msg.linear_acceleration_covariance[8] = 0.01;
        }
        imu_pub_->publish(msg);
        // 重置标志，防止重复发布相同数据（可选）
        // imu_data_.has_acc = imu_data_.has_gyro = imu_data_.has_quat = false;
    }

    // 发布GPS消息
    void publish_gps()
    {
        if (!gps_data_.has_data) return;
        auto msg = sensor_msgs::msg::NavSatFix();
        msg.header.stamp = this->now();
        msg.header.frame_id = "gps_link";
        msg.latitude = gps_data_.lat;
        msg.longitude = gps_data_.lon;
        msg.altitude = 0.0; // 可扩展气压高度
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        gps_pub_->publish(msg);
        gps_data_.has_data = false;
    }

    // 串口数据读取与解析主循环
    void read_serial_callback()
    {
        uint8_t buf[256];
        int n = read(serial_fd_, buf, sizeof(buf));
        if (n > 0) {
            buffer_.insert(buffer_.end(), buf, buf + n);
        }

        // 解析数据包
        while (buffer_.size() >= 2) {
            auto it = std::find(buffer_.begin(), buffer_.end(), 0x55);
            if (it == buffer_.end()) {
                buffer_.clear();
                break;
            }
            buffer_.erase(buffer_.begin(), it);
            if (buffer_.size() < 2) break;

            uint8_t type = buffer_[1];
            size_t pkt_len = 0;
            switch (type) {
                case 0x51: case 0x52: case 0x53: case 0x57: case 0x59:
                    pkt_len = 11;   // 这些包长度为11字节
                    break;
                default:
                    // 未知类型，跳过包头
                    buffer_.erase(buffer_.begin());
                    continue;
            }
            if (buffer_.size() < pkt_len) break;

            std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + pkt_len);
            uint8_t calc_sum = calc_checksum(packet);
            if (calc_sum != packet.back()) {
                // 校验失败，丢弃包头并继续
                buffer_.erase(buffer_.begin());
                continue;
            }

            // 根据类型解析
            switch (type) {
                case 0x51: parse_accel(packet); break;
                case 0x52: parse_gyro(packet); break;
                case 0x59: parse_quaternion(packet); publish_imu(); break;
                case 0x57: parse_gps(packet); publish_gps(); break;
                default: break;
            }
            buffer_.erase(buffer_.begin(), buffer_.begin() + pkt_len);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUGPSParser>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}