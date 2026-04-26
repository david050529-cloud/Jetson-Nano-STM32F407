cmake_minimum_required(VERSION 3.8)
project(vision_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# 摄像头节点（发布原始图像，由 image_transport 压缩）
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp sensor_msgs cv_bridge OpenCV image_transport)

# YOLO 检测节点（C++，订阅并自动解压 H.264）
add_executable(yolo_video_subscriber src/yolo_video_subscriber.cpp)
ament_target_dependencies(yolo_video_subscriber
  rclcpp sensor_msgs cv_bridge OpenCV image_transport)

# 安装可执行文件
install(TARGETS camera_node yolo_video_subscriber
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()