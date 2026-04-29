cmake_minimum_required(VERSION 3.8)
project(imu_gps_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(imu_gps_parser_node src/imu_gps_parser_node.cpp)
target_include_directories(imu_gps_parser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_gps_parser_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(imu_gps_parser_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

add_executable(imu_gps_subscriber_node src/imu_gps_subscriber_node.cpp)
target_compile_features(imu_gps_subscriber_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(imu_gps_subscriber_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  imu_gps_parser_node
  imu_gps_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()