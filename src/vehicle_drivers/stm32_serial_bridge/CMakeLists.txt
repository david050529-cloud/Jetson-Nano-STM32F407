cmake_minimum_required(VERSION 3.8)
project(stm32_serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MotorCommand.msg"
  "msg/Telemetry.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

add_executable(stm32_serial_bridge_node src/stm32_serial_bridge_node.cpp)
target_include_directories(stm32_serial_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(stm32_serial_bridge_node
  rclcpp
  serial
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stm32_serial_bridge_node
  ${cpp_typesupport_target}
)

install(TARGETS stm32_serial_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rclcpp serial rosidl_default_runtime)
ament_package()