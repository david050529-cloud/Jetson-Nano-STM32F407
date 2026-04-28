cmake_minimum_required(VERSION 3.8)
project(imu_gps_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(serial_driver REQUIRED)

# IMU 发布节点
add_executable(imu_publisher_node src/imu_publisher_node.cpp)
target_include_directories(imu_publisher_node PRIVATE
  ${serial_driver_INCLUDE_DIRS})
ament_target_dependencies(imu_publisher_node
  rclcpp
  sensor_msgs
  serial_driver
)

# GPS 发布节点
add_executable(gps_publisher_node src/gps_publisher_node.cpp)
ament_target_dependencies(gps_publisher_node
  rclcpp
  std_msgs
  sensor_msgs
  serial
)

install(TARGETS
  imu_publisher_node
  gps_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()