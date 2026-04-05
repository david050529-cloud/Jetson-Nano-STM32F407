cmake_minimum_required(VERSION 3.8)
project(gps_node)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(serial REQUIRED)

# GPS发布节点
add_executable(gps_publisher_node src/gps_publisher_node.cpp)
ament_target_dependencies(gps_publisher_node rclcpp std_msgs sensor_msgs serial)

# GPS订阅节点
add_executable(gps_subscriber_node src/gps_subscriber_node.cpp)
ament_target_dependencies(gps_subscriber_node rclcpp std_msgs sensor_msgs serial)

# 安装可执行文件
install(TARGETS
  gps_publisher_node
  gps_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()