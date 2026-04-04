cmake_minimum_required(VERSION 3.8)
project(imu_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_driver REQUIRED)

# IMU驱动节点
add_executable(imu_driver_node src/imu_publisher_node.cpp)
ament_target_dependencies(imu_driver_node 
  rclcpp 
  sensor_msgs 
  serial_driver
)

# IMU客户端节点
add_executable(imu_client_node src/imu_client_node.cpp)
ament_target_dependencies(imu_client_node 
  rclcpp 
  sensor_msgs
)

# 安装可执行文件
install(TARGETS
  imu_driver_node
  imu_client_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()