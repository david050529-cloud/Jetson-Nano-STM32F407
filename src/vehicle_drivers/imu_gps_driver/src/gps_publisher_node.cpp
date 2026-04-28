#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>

class GpsPublisherNode : public rclcpp::Node
{
public:
    GpsPublisherNode() : Node("gps_publisher_node")
    {
        this->declare_parameter<std::string>("port", "/dev/GPS");
        this->declare_parameter<int>("baudrate", 9600);
        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baudrate").as_int();

        try
        {
            serial_.setPort(port);
            serial_.setBaudrate(baud);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(timeout);
            serial_.open();
            RCLCPP_INFO(this->get_logger(), "GPS serial port %s opened at %d baud", port.c_str(), baud);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GPS port: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        status_pub_ = this->create_publisher<sensor_msgs::msg::NavSatStatus>("/gps/status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GpsPublisherNode::readAndPublish, this));
    }

private:
    void readAndPublish()
    {
        if (!serial_.isOpen()) return;
        std::string line;
        try
        {
            line = serial_.readline(256, "\n");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "GPS read error: %s", e.what());
            return;
        }
        if (line.empty()) return;
        if (line.back() == '\n') line.pop_back();
        if (line.empty()) return;

        if (line.find("GGA") != std::string::npos)
            parseGGA(line);
    }

    void parseGGA(const std::string &nmea)
    {
        std::vector<std::string> fields;
        std::stringstream ss(nmea);
        std::string field;
        while (std::getline(ss, field, ','))
            fields.push_back(field);
        if (fields.size() < 10) return;

        double lat = 0.0, lon = 0.0, alt = 0.0;
        int fix_quality = 0;

        if (fields.size() > 6 && !fields[6].empty())
            fix_quality = std::stoi(fields[6]);
        if (fields.size() > 3 && !fields[2].empty() && !fields[3].empty())
            lat = convertNmeaCoordinate(fields[2], fields[3][0]);
        if (fields.size() > 5 && !fields[4].empty() && !fields[5].empty())
            lon = convertNmeaCoordinate(fields[4], fields[5][0]);
        if (fields.size() > 9 && !fields[9].empty())
            alt = std::stod(fields[9]);

        auto fix_msg = sensor_msgs::msg::NavSatFix();
        fix_msg.header.stamp = this->now();
        fix_msg.header.frame_id = "gps";
        fix_msg.latitude = lat;
        fix_msg.longitude = lon;
        fix_msg.altitude = alt;
        fix_msg.status.status = (fix_quality > 0) ?
            sensor_msgs::msg::NavSatStatus::STATUS_FIX :
            sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        fix_msg.position_covariance[0] = 1.0;
        fix_msg.position_covariance[4] = 1.0;
        fix_msg.position_covariance[8] = 1.0;
        fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

        fix_pub_->publish(fix_msg);
        RCLCPP_INFO(this->get_logger(), "GPS fix: lat=%.6f, lon=%.6f, alt=%.1f, quality=%d",
                    lat, lon, alt, fix_quality);
    }

    double convertNmeaCoordinate(const std::string &coord, char dir)
    {
        if (coord.empty()) return 0.0;
        size_t dotPos = coord.find('.');
        if (dotPos == std::string::npos) return 0.0;
        double minutes = std::stod(coord.substr(dotPos - 2));
        double degrees = std::stod(coord.substr(0, dotPos - 2));
        double decimal = degrees + minutes / 60.0;
        if (dir == 'S' || dir == 'W') decimal = -decimal;
        return decimal;
    }

    serial::Serial serial_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}