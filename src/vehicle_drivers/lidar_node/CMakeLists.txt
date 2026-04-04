cmake_minimum_required(VERSION 3.8)
project(lidar_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(    
    -Wall 
    -Wextra 
    -Wno-unused-parameter
    -Wno-unused-function
    -Wno-unused-value
    -Wno-cast-function-type
    -Wno-ignored-qualifiers)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(serial_driver REQUIRED)
find_package(Python3 REQUIRED)

# 设置SDK路径
set(SDK_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk)
set(SDK_INCLUDE_DIR ${SDK_BASE_DIR}/include)
set(SDK_SRC_DIR ${SDK_BASE_DIR}/src)

message(STATUS "SDK base dir: ${SDK_BASE_DIR}")
message(STATUS "SDK include dir: ${SDK_INCLUDE_DIR}")
message(STATUS "SDK src dir: ${SDK_SRC_DIR}")

# 添加头文件路径
include_directories(
    include
    ${SDK_INCLUDE_DIR}
    ${SDK_SRC_DIR}
    ${SDK_SRC_DIR}/arch/linux
    ${SDK_SRC_DIR}/hal
    ${SDK_SRC_DIR}/dataunpacker
    ${SDK_SRC_DIR}/dataunpacker/unpacker
)

# 查找Linux平台的源文件
file(GLOB_RECURSE SDK_SOURCES_LINUX
    ${SDK_SRC_DIR}/arch/linux/*.cpp
    ${SDK_SRC_DIR}/hal/*.cpp
    ${SDK_SRC_DIR}/*.cpp
    ${SDK_SRC_DIR}/dataunpacker/*.cpp
    ${SDK_SRC_DIR}/dataunpacker/unpacker/*.cpp
)

# 过滤掉其他平台文件
list(FILTER SDK_SOURCES_LINUX EXCLUDE REGEX ".*/arch/win32/.*")
list(FILTER SDK_SOURCES_LINUX EXCLUDE REGEX ".*/arch/win7/.*")
list(FILTER SDK_SOURCES_LINUX EXCLUDE REGEX ".*/arch/macOS/.*")
list(FILTER SDK_SOURCES_LINUX EXCLUDE REGEX ".*/arch/ios/.*")
list(FILTER SDK_SOURCES_LINUX EXCLUDE REGEX ".*/arch/android/.*")

message(STATUS "Found Linux SDK sources: ${SDK_SOURCES_LINUX}")

# lidar_cline_node 可执行文件
add_executable(lidar_cline_node
  src/lidar_cline_node.cpp
  ${SDK_SOURCES_LINUX}
)

target_include_directories(lidar_cline_node PRIVATE
    ${SDK_INCLUDE_DIR}
    ${SDK_SRC_DIR}
    ${SDK_SRC_DIR}/arch/linux
    ${SDK_SRC_DIR}/hal
    ${SDK_SRC_DIR}/dataunpacker
    ${SDK_SRC_DIR}/dataunpacker/unpacker
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(lidar_cline_node
  rclcpp
  sensor_msgs
  std_srvs
  serial_driver
)

target_link_libraries(lidar_cline_node 
    pthread
    rt
    dl
)

# lidar_publisher_node 可执行文件
add_executable(lidar_publisher_node
  src/lidar_publisher_node.cpp
  ${SDK_SOURCES_LINUX}
)

target_include_directories(lidar_publisher_node PRIVATE
    ${SDK_INCLUDE_DIR}
    ${SDK_SRC_DIR}
    ${SDK_SRC_DIR}/arch/linux
    ${SDK_SRC_DIR}/hal
    ${SDK_SRC_DIR}/dataunpacker
    ${SDK_SRC_DIR}/dataunpacker/unpacker
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(lidar_publisher_node
  rclcpp
  sensor_msgs
  std_srvs
  serial_driver
)

target_link_libraries(lidar_publisher_node 
    pthread
    rt
    dl
)

# 安装可执行文件
install(TARGETS
    lidar_cline_node
    lidar_publisher_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

# 可选：安装Python模块（如果需要）
install(DIRECTORY scripts
    DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()