/*
 * 雷达代码，根据例程进行修改;
 * 基于思岚科技(SLAMTEC)激光雷达SDK，将雷达数据转换为ROS2的LaserScan消息并发布;
 * 根据例程进行修改
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sl_lidar.h"
#include <math.h>
#include "signal.h"
#include <signal.h>

#ifndef _countof
// 计算数组元素个数的宏定义
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

// 角度转弧度宏
#define DEG2RAD(x) ((x) * M_PI / 180.)

// ROS2版本标识
#define ROS2VERSION "humble"

using namespace sl;
// 使用思岚SDK命名空间

bool need_exit = false;
// 程序退出标志，用于信号处理

/**
 * @brief 激光雷达发布节点类
 *
 * 负责初始化激光雷达、配置参数、采集数据并发布为LaserScan消息
 */

class LidarPublisherNode : public rclcpp::Node
{

public:
    /**
     * @brief 构造函数
     * 创建名为"lidar_publisher"的ROS2节点，并初始化scan话题发布器
     */
    LidarPublisherNode() : Node("lidar_publisher")
    {
        // 创建scan话题发布器，队列大小为10
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(1000)));
    }

private:
    /**
     * @brief 初始化ROS2参数
     *
     * 声明并获取所有配置参数，包括连接方式、网络配置、串口配置等
     */
    void init_params()
    {
        // 声明参数并设置默认值
        this->declare_parameter<std::string>("channel_type", "serial");      // 连接类型：serial/tcp/udp
        this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");       // TCP连接IP地址
        this->declare_parameter<int>("tcp_port", 20108);                     // TCP连接端口
        this->declare_parameter<std::string>("udp_ip", "192.168.11.2");      // UDP连接IP地址
        this->declare_parameter<int>("udp_port", 8089);                      // UDP连接端口
        this->declare_parameter<std::string>("serial_port", "/dev/Lidar"); // 串口设备路径
        this->declare_parameter<int>("serial_baudrate", 1000000);            // 串口波特率
        this->declare_parameter<std::string>("frame_id", "laser_frame");     // 坐标系名称
        this->declare_parameter<bool>("inverted", false);                    // 是否反转数据方向
        this->declare_parameter<bool>("angle_compensate", false);            // 是否进行角度补偿
        this->declare_parameter<std::string>("scan_mode", std::string());    // 扫描模式
        this->declare_parameter<float>("scan_frequency", 10);                // 扫描频率

        // 获取参数值
        this->get_parameter_or<std::string>("channel_type", channel_type, "serial");
        this->get_parameter_or<std::string>("tcp_ip", tcp_ip, "192.168.0.7");
        this->get_parameter_or<int>("tcp_port", tcp_port, 20108);
        this->get_parameter_or<std::string>("udp_ip", udp_ip, "192.168.11.2");
        this->get_parameter_or<int>("udp_port", udp_port, 8089);
        this->get_parameter_or<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
        this->get_parameter_or<int>("serial_baudrate", serial_baudrate, 1000000); // A1/A2用1000000，A3用256000
        this->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
        this->get_parameter_or<bool>("inverted", inverted, false);
        this->get_parameter_or<bool>("angle_compensate", angle_compensate, false);
        this->get_parameter_or<std::string>("scan_mode", scan_mode, std::string());

        // UDP模式默认扫描频率20Hz，其他模式默认10Hz
        if (channel_type == "udp")
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 20.0);
        else
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 10.0);
    }

    /**
     * @brief 获取激光雷达设备信息
     *
     * @param drv 激光雷达驱动指针
     * @return true 获取成功
     * @return false 获取失败
     */

    bool getSLLIDARDeviceInfo(ILidarDriver *drv)
    {
        sl_result op_result;
        sl_lidar_response_device_info_t devinfo;

        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result))
        {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT)
            {
                RCLCPP_ERROR(this->get_logger(), "Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! ");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Error, unexpected error, code: %x", op_result);
            }
            return false;
        }

        // 打印设备序列号（16字节转16进制字符串）
        char sn_str[37] = {'\0'};
        for (int pos = 0; pos < 16; ++pos)
        {
            sprintf(sn_str + (pos * 2), "%02X", devinfo.serialnum[pos]);
        }
        RCLCPP_INFO(this->get_logger(), "SLLidar S/N: %s", sn_str);
        RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d", (int)devinfo.hardware_version);
        return true;
    }

    /**
     * @brief 检查激光雷达健康状态
     *
     * @param drv 激光雷达驱动指针
     * @return true 设备正常
     * @return false 设备异常
     */

    bool checkSLLIDARHealth(ILidarDriver *drv)
    {
        sl_result op_result;
        sl_lidar_response_device_health_t healthinfo;
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result))
        {
            RCLCPP_INFO(this->get_logger(), "SLLidar health status : %d", healthinfo.status);
            switch (healthinfo.status)
            {
            case SL_LIDAR_STATUS_OK:
                RCLCPP_INFO(this->get_logger(), "SLLidar health status : OK.");
                return true;
            case SL_LIDAR_STATUS_WARNING:
                RCLCPP_INFO(this->get_logger(), "SLLidar health status : Warning.");
                return true;
            case SL_LIDAR_STATUS_ERROR:
                RCLCPP_ERROR(this->get_logger(), "Error, SLLidar internal error detected. Please reboot the device to retry.");
                return false;
            default:
                RCLCPP_ERROR(this->get_logger(), "Error, Unknown internal error detected. Please reboot the device to retry.");
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve SLLidar health code: %x", op_result);
            return false;
        }
    }

    /**
     * @brief 停止电机服务回调函数
     *
     * @param req 空请求
     * @param res 空响应
     * @return true 停止成功
     * @return false 停止失败
     */

    bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if (!drv)
            return false;

        RCLCPP_DEBUG(this->get_logger(), "Stop motor");
        drv->setMotorSpeed(0); // 设置电机转速为0
        return true;
    }

    /**
     * @brief 启动电机服务回调函数
     *
     * @param req 空请求
     * @param res 空响应
     * @return true 启动成功
     * @return false 启动失败
     */

    bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                     std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if (!drv)
            return false;
        if (drv->isConnected())
        {
            RCLCPP_DEBUG(this->get_logger(), "Start motor");
            sl_result ans = drv->setMotorSpeed(); // 启动电机（默认转速）
            if (SL_IS_FAIL(ans))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to start motor: %08x", ans);
                return false;
            }

            ans = drv->startScan(0, 1);
            if (SL_IS_FAIL(ans))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to start scan: %08x", ans);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "lost connection");
            return false;
        }

        return true;
    }

    /**
     * @brief 获取测量点的角度值
     *
     * @param node 测量节点
     * @return float 角度值（度）
     */

    static float getAngle(const sl_lidar_response_measurement_node_hq_t &node)
    {
        // angle_z_q14 是Q14格式的角度值，转换公式：角度 = angle_z_q14 * 90 / 16384
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    /**
     * @brief 发布LaserScan消息
     *
     * @param pub 发布器指针
     * @param nodes 测量数据节点数组
     * @param node_count 节点数量
     * @param start 扫描开始时间
     * @param scan_time 扫描耗时
     * @param inverted 是否反转
     * @param angle_min 最小角度
     * @param angle_max 最大角度
     * @param max_distance 最大测距距离
     * @param frame_id 坐标系ID
     */

    void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &pub,
                      sl_lidar_response_measurement_node_hq_t *nodes,
                      size_t node_count, rclcpp::Time start,
                      double scan_time, bool inverted,
                      float angle_min, float angle_max,
                      float max_distance,
                      std::string frame_id)
    {
        static int scan_count = 0;
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp = start;       // 设置时间戳
        scan_msg->header.frame_id = frame_id; // 设置坐标系
        scan_count++;

        // 根据反转标志和角度范围计算消息中的角度参数
        bool reversed = (angle_max > angle_min);
        if (reversed)
        {
            scan_msg->angle_min = M_PI - angle_max;
            scan_msg->angle_max = M_PI - angle_min;
        }
        else
        {
            scan_msg->angle_min = M_PI - angle_min;
            scan_msg->angle_max = M_PI - angle_max;
        }
        // 计算角度增量
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count - 1);

        scan_msg->scan_time = scan_time;                                 // 扫描周期
        scan_msg->time_increment = scan_time / (double)(node_count - 1); // 每点时间增量
        scan_msg->range_min = 0.15;                                      // 最小测距距离（米）
        scan_msg->range_max = max_distance;                              // 最大测距距离（米）

        scan_msg->intensities.resize(node_count); // 强度数组
        scan_msg->ranges.resize(node_count);      // 距离数组

        // 判断是否需要反转数据顺序
        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
        if (!reverse_data)
        {
            for (size_t i = 0; i < node_count; i++)
            {
                // dist_mm_q2是Q2格式的距离值（mm），转换为米：除以4再除以1000
                float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
                if (read_value == 0.0)
                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity(); // 无效点设为无穷大
                else
                    scan_msg->ranges[i] = read_value;
                scan_msg->intensities[i] = (float)(nodes[i].quality >> 2); // 强度值（quality高2位无用）
            }
        }
        else
        {
            // 反转顺序发布
            for (size_t i = 0; i < node_count; i++)
            {
                float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
                if (read_value == 0.0)
                    scan_msg->ranges[node_count - 1 - i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[node_count - 1 - i] = read_value;
                scan_msg->intensities[node_count - 1 - i] = (float)(nodes[i].quality >> 2);
            }
        }

        pub->publish(*scan_msg); // 发布消息
    }

public:
    /**
     * @brief 主工作循环
     *
     * 初始化雷达、连接设备、启动扫描，并循环获取数据发布
     *
     * @return int 0成功，非0失败
     */
    int work_loop()
    {
        init_params(); // 初始化参数
        // 打印SDK版本信息
        int ver_major = SL_LIDAR_SDK_VERSION_MAJOR;
        int ver_minor = SL_LIDAR_SDK_VERSION_MINOR;
        int ver_patch = SL_LIDAR_SDK_VERSION_PATCH;
        RCLCPP_INFO(this->get_logger(), "SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:" ROS2VERSION ", SLLIDAR SDK Version:%d.%d.%d", ver_major, ver_minor, ver_patch);

        sl_result op_result;

        // 创建驱动实例
        drv = *createLidarDriver();
        IChannel *_channel;
        // 根据连接类型创建对应的通信通道
        if (channel_type == "tcp")
        {
            _channel = *createTcpChannel(tcp_ip, tcp_port);
        }
        else if (channel_type == "udp")
        {
            _channel = *createUdpChannel(udp_ip, udp_port);
        }
        else
        {
            _channel = *createSerialPortChannel(serial_port, serial_baudrate);
        }
        // 连接设备
        if (SL_IS_FAIL((drv)->connect(_channel)))
        {
            if (channel_type == "tcp")
            {
                RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to the ip addr  %s with the tcp port %s.", tcp_ip.c_str(), std::to_string(tcp_port).c_str());
            }
            else if (channel_type == "udp")
            {
                RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to the ip addr  %s with the udp port %s.", udp_ip.c_str(), std::to_string(udp_port).c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Error, cannot bind to the specified serial port %s.", serial_port.c_str());
            }
            delete drv;
            return -1;
        }

        // 获取设备信息
        if (!getSLLIDARDeviceInfo(drv))
        {
            return -1;
        }

        // 检查设备健康状态
        if (!checkSLLIDARHealth(drv))
        {
            return -1;
        }
        // 创建服务：停止电机
        stop_motor_service = this->create_service<std_srvs::srv::Empty>("stop_motor",
                                                                        std::bind(&LidarPublisherNode::stop_motor, this, std::placeholders::_1, std::placeholders::_2));

        // 创建服务：启动电机
        start_motor_service = this->create_service<std_srvs::srv::Empty>("start_motor",
                                                                         std::bind(&LidarPublisherNode::start_motor, this, std::placeholders::_1, std::placeholders::_2));

        drv->setMotorSpeed(); // 启动电机

        LidarScanMode current_scan_mode;
        // 根据是否指定扫描模式来选择启动方式
        if (scan_mode.empty())
        {
            // 使用典型扫描模式启动
            op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
        }
        else
        {
            // 获取所有支持的扫描模式
            std::vector<LidarScanMode> allSupportedScanModes;
            op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

            if (SL_IS_OK(op_result))
            {
                sl_u16 selectedScanMode = sl_u16(-1);
                // 查找用户指定的扫描模式
                for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++)
                {
                    if (iter->scan_mode == scan_mode)
                    {
                        selectedScanMode = iter->id;
                        break;
                    }
                }

                if (selectedScanMode == sl_u16(-1))
                {
                    // 未找到指定模式，打印所有支持的模式
                    RCLCPP_ERROR(this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                    for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++)
                    {
                        RCLCPP_ERROR(this->get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK", iter->scan_mode,
                                     iter->max_distance, (1000 / iter->us_per_sample));
                    }
                    op_result = SL_RESULT_OPERATION_FAIL;
                }
                else
                {
                    // 使用指定模式启动快速扫描
                    op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
                }
            }
        }
        // 计算角度补偿参数
        if (SL_IS_OK(op_result))
        {
            // 根据扫描频率和每点采样时间计算每圈点数和角度补偿倍数
            int points_per_circle = (int)(1000 * 1000 / current_scan_mode.us_per_sample / scan_frequency);
            angle_compensate_multiple = points_per_circle / 360.0 + 1; // 每度补偿点数
            if (angle_compensate_multiple < 1)
                angle_compensate_multiple = 1.0;
            max_distance = (float)current_scan_mode.max_distance; // 最大测距距离
            RCLCPP_INFO(this->get_logger(), "current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ",
                        current_scan_mode.scan_mode, (int)(1000 / current_scan_mode.us_per_sample + 0.5), max_distance, scan_frequency);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Can not start scan: %08x!", op_result);
        }

        rclcpp::Time start_scan_time;
        rclcpp::Time end_scan_time;
        double scan_duration;
        // 主循环：获取数据并发布
        while (rclcpp::ok() && !need_exit)
        {
            sl_lidar_response_measurement_node_hq_t nodes[8192]; // 存储原始数据
            size_t count = _countof(nodes);

            start_scan_time = this->now();                               // 记录开始时间
            op_result = drv->grabScanDataHq(nodes, count);               // 获取扫描数据
            end_scan_time = this->now();                                 // 记录结束时间
            scan_duration = (end_scan_time - start_scan_time).seconds(); // 计算扫描耗时

            if (op_result == SL_RESULT_OK)
            {
                // 整理扫描数据（按角度排序）
                op_result = drv->ascendScanData(nodes, count);
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);
                if (op_result == SL_RESULT_OK)
                {
                    if (angle_compensate)
                    {
                        // 角度补偿模式：将数据插值到每个整角度上
                        const int angle_compensate_nodes_count = 360 * angle_compensate_multiple;
                        int angle_compensate_offset = 0;
                        auto angle_compensate_nodes = new sl_lidar_response_measurement_node_hq_t[angle_compensate_nodes_count];
                        memset(angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(sl_lidar_response_measurement_node_hq_t));

                        size_t i = 0, j = 0;
                        for (; i < count; i++)
                        {
                            if (nodes[i].dist_mm_q2 != 0) // 有效数据点
                            {
                                float angle = getAngle(nodes[i]);
                                int angle_value = (int)(angle * angle_compensate_multiple);
                                if ((angle_value - angle_compensate_offset) < 0)
                                    angle_compensate_offset = angle_value;
                                // 将当前点填充到补偿数组的多个位置
                                for (j = 0; j < angle_compensate_multiple; j++)
                                {
                                    int angle_compensate_nodes_index = angle_value - angle_compensate_offset + j;
                                    if (angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                        angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
                                    angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                                }
                            }
                        }
                        // 发布补偿后的数据
                        publish_scan(scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                                     start_scan_time, scan_duration, inverted,
                                     angle_min, angle_max, max_distance,
                                     frame_id);

                        if (angle_compensate_nodes)
                        {
                            delete[] angle_compensate_nodes;
                            angle_compensate_nodes = nullptr;
                        }
                    }
                    else
                    {
                        // 非补偿模式：只发布有效数据区域
                        int start_node = 0, end_node = 0;
                        int i = 0;
                        // 找到第一个有效数据点
                        while (nodes[i++].dist_mm_q2 == 0)
                            ;
                        start_node = i - 1;
                        i = count - 1;
                        // 找到最后一个有效数据点
                        while (nodes[i--].dist_mm_q2 == 0)
                            ;
                        end_node = i + 1;

                        angle_min = DEG2RAD(getAngle(nodes[start_node]));
                        angle_max = DEG2RAD(getAngle(nodes[end_node]));

                        publish_scan(scan_pub, &nodes[start_node], end_node - start_node + 1,
                                     start_scan_time, scan_duration, inverted,
                                     angle_min, angle_max, max_distance,
                                     frame_id);
                    }
                }
                else if (op_result == SL_RESULT_OPERATION_FAIL)
                {
                    // 数据无效，发布所有数据（角度范围为0-359度）
                    float angle_min = DEG2RAD(0.0f);
                    float angle_max = DEG2RAD(359.0f);
                    publish_scan(scan_pub, nodes, count,
                                 start_scan_time, scan_duration, inverted,
                                 angle_min, angle_max, max_distance,
                                 frame_id);
                }
            }

            rclcpp::spin_some(shared_from_this()); // 处理回调
        }

        // done!
        drv->setMotorSpeed(0); // 停止电机
        drv->stop();           // 停止扫描
        RCLCPP_INFO(this->get_logger(), "Stop motor");

        return 0;
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;   // scan话题发布器
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service; // 启动电机服务
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service;  // 停止电机服务

    // 配置参数
    std::string channel_type;             // 连接类型：serial/tcp/udp
    std::string tcp_ip;                   // TCP IP地址
    std::string udp_ip;                   // UDP IP地址
    std::string serial_port;              // 串口设备路径
    int tcp_port = 20108;                 // TCP端口
    int udp_port = 8089;                  // UDP端口
    int serial_baudrate = 115200;         // 串口波特率
    std::string frame_id;                 // 坐标系名称
    bool inverted = false;                // 是否反转
    bool angle_compensate = true;         // 是否角度补偿
    float max_distance = 8.0;             // 最大测距距离（米）
    size_t angle_compensate_multiple = 1; // 角度补偿倍数（每度点数）
    std::string scan_mode;                // 扫描模式
    float scan_frequency;                 // 扫描频率（Hz）

    ILidarDriver *drv; // 激光雷达驱动指针
};

/**
 * @brief 退出信号处理函数
 *
 * @param sig 信号编号
 */

void ExitHandler(int sig)
{
    (void)sig;
    need_exit = true; // 设置退出标志
}

/**
 * @brief 主函数
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 返回值
 */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                           // 初始化ROS2
    auto lidar_publisher_node = std::make_shared<LidarPublisherNode>(); // 创建节点实例
    signal(SIGINT, ExitHandler);                                        // 注册SIGINT信号处理函数
    int ret = lidar_publisher_node->work_loop();                        // 运行主循环
    rclcpp::shutdown();                                                 // 关闭ROS2
    return ret;
}