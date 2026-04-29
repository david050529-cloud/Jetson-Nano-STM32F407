# Jetson-Nano-STM32F407

该项目使用激光雷达、摄像头、IMU传感器以及GPS来获取状态信息以及环境信息，向MCU发送指令来控制车辆的运动，包括速度和方向。\
采用Jetson Nano作为上位机，STM32F407开发板作为下位机，二者之间使用UART进行通信。

## GPS节点

实现GPS数据的发布与订阅；\
目前需要在室外进行验证；

## IMU节点

实现读取IMU传感器的数据，同时进行发布与订阅；\
尚未开发客户端界面战术数据；

## 雷达节点

实现雷达使用;目前启用翻转。\
实际激光雷安装位置可能需要进行优化；

## 摄像头节点

需要提高帧率较低的问题，能够实现检测，\下一步需要针对实际情景训练模型；

ros2 launch vision_node vision.launch.py\
ros2 run rqt_image_view rqt_image_view

## USART通信节点

下一步构建通信功能包，实现上位机与下位机之间的通信，通过上位机对下位机发布指令控制编码电机的转速，下位机实时返回编码电机转速，上位机推断当前速度；上位机也需要发布对转向舵机的控制，也需要发布对云台的控制（后期添加）；\
/dev/ttyACM0\
/dev/ttyTSH1

## C/C++配置

${workspaceFolder}/**\
/opt/ros/${ROS_DISTRO}/include**\
/opt/ros/humble/include/rosidl_runtime_cpp\
/opt/ros/humble/include/rclcpp\
/opt/ros/humble/include/std_msgs

## vehicle_bringup

- **功能**:
  - 提供车辆系统的启动和配置功能。
  - 包含`vehicle.launch.py`，用于启动车辆相关的节点。

- **用途**:
  - 作为系统的入口点，协调其他功能包的启动。
  - `ros2 launch vehicle_bringup vehicle.launch.py`，启动整个车辆系统。

## vehicle_control

- **功能**:
  - 实现车辆的控制逻辑。
  - 包含`cmd_converter.py`，可能用于将高层次的控制命令（如路径规划的结果）转换为车辆底层的控制指令。

- **用途**:
  - 控制车辆的运动，包括速度和方向。

## vehicle_drivers

- **gps_node**:
  - 提供GPS数据的发布和订阅功能。
    - 包含`gps_publisher_node.cpp`和`gps_subscriber_node.cpp`，分别用于发布和订阅GPS数据。
    - `ros2 launch gps_node gps.launch.py port:=/dev/ttyUSB0 baudrate:=9600`
    - 查看完整的 NavSatFix 消息`ros2 topic echo /gps/fix`
    - 仅查看经纬度（使用 yaml 格式过滤）`ros2 topic echo /gps/fix | grep -E "latitude|longitude|altitude"`
    - `rviz2`，添加 NavSatFix 显示，选择 /gps/fix 主题，设置坐标系为 GPS。

  - **imu_node**:
    - 提供IMU（惯性测量单元）数据的驱动和发布功能。
    - 包含`imu_publisher_node.cpp`和 `imu_client_node.cpp`，分别用于发布IMU数据和与IMU设备通信。
    - 包含`bind_usb.sh`和`imu_usb.rules`，可能用于配置IMU设备的USB连接。
    - `ros2 launch imu_node imu.launch.py`
    - `ros2 topic echo /imu/data_raw`
    - `ros2 run rqt_plot rqt_plot`
    - `sudo apt install ros-${ROS_DISTRO}-rqt-plot`
  - **lidar_node**:
    - 提供激光雷达（LiDAR）数据的驱动和发布功能。
    - 包含`lidar.launch.py`和`sllidar_ros2.rviz`，用于启动LiDAR节点和可视化LiDAR数据。
    - `ros2 launch lidar_node lidar.launch.py`
    - `rviz2`
  - **stm32_serial_bridge**:
    - 提供与STM32微控制器的串口通信功能。
    - `ros2 launch stm32_serial_bridge bridge_launch.py`
    - `ros2 topic echo /telemetry`
    - `ros2 topic pub /motor_commands stm32_serial_bridge/msg/MotorCommand "{motor1_target_rps: 1.0, motor2_target_rps: 1.0, servo_angle: 90}" -1`
  - **vision_node**
    - 提供视觉相关的功能（具体实现未完全展示）。
    - 可能用于处理摄像头数据，支持感知或定位功能。
    - `ros2 launch vision_package vision.launch.py`
    - `ros2 run rqt_image_view rqt_image_view`
    - 修改 `CMakeLists.txt`，取消`yolo_video_subscriber`的注释，然后重新编译
        `colcon build --packages-select vision_package`
        `ros2 run vision_package yolo_video_subscriber`

## vehicle_localization

- **功能**:
  - 实现车辆的定位功能。
  - 包含`ekf.yaml`配置文件，可能用于扩展卡尔曼滤波器（EKF）的参数配置。
  - 包含`localization.launch.py`，用于启动定位相关的节点。

- **用途**:
  - 结合传感器数据（如GPS和IMU）实现车辆的实时定位。
  - `ros2 launch vehicle_localization localization.launch.py`，启动定位系统。
  - `ros2 topic echo /odometry/filtered`，查看融合后的里程计数据。

## vehicle_perception

- **功能**:
  - 实现环境感知功能。
  - 包含`obstacle_detector.py`、`road_detector.py` 和`traffic_sign_detector.py`，分别用于检测障碍物、道路和交通标志。

- **用途**:
  - 提供车辆对周围环境的感知能力，支持自动驾驶决策。
  - `ros2 launch vehicle_perception perception.launch.py`，启动感知系统。`
  - `ros2 run rqt_image_view rqt_image_view`，查看摄像头数据和感知结果。
  - `ros2 run vehicle_perception traffic_sign_detector`，运行交通标志检测节点。
  - `ros2 run vehicle_perception road_detector`，运行道路检测节点。
  - `ros2 run vehicle_perception obstacle_detector`，运行障碍物检测节点

## vehicle_planning

- **功能**:
  - 实现路径规划功能。
  - 包含`global_planner.py`和`local_planner.py`，分别用于全局路径规划和局部路径规划。
  - 包含`path_planner.py`，可能是路径规划的主程序。
  - 包含`waypoint_parser.py`，用于解析路径点。
  - 包含`path_planning.launch.py`，用于启动路径规划相关的节点。

- **用途**:
  - 根据感知和定位数据生成车辆的行驶路径。
  - `ros2 launch vehicle_planning path_planning.launch.py`，启动路径规划系统。
  - `ros2 run vehicle_planning path_planner`，运行路径规划节点。
  - `ros2 run vehicle_planning global_planner`，运行全局路径规划节点。
  - `ros2 run vehicle_planning local_planner`，运行局部路径规划节点。
  - `ros2 topic echo /planning/global_path`，查看全局路径数据。
  - `ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 0.0}}}"`，发布一个新的目标位置。

imu_device_udev_rules.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="IMU"

lidar_device_udev_rules.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="Lidar"

gps_device_udev_rules.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="GPS"

stm32_device_udev_rules.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", SYMLINK+="STM32"

Unable to load plugin for transport 'image_transport/compressed_sub', error string:
According to the loaded plugin descriptions the class image_transport/compressed_sub with base class type image_transport::SubscriberPlugin does not exist. Declared types are  image_transport/ffmpeg_sub image_transport/raw_sub