cmake_minimum_required(VERSION 3.8)
project(stm32_serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# 添加 rosidl 生成器（必须）
find_package(rosidl_default_generators REQUIRED)

# 添加自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorServoCmd.msg"
  "msg/Telemetry.msg"
  DEPENDENCIES std_msgs
)

# 添加可执行文件
add_executable(stm32_serial_bridge_node src/stm32_serial_bridge_node.cpp)
ament_target_dependencies(stm32_serial_bridge
  rclcpp
  std_msgs
  serial
)

# 替换弃用的 rosidl_target_interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stm32_serial_bridge_node ${cpp_typesupport_target})

# 安装
install(TARGETS stm32_serial_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()