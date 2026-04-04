cmake_minimum_required(VERSION 3.8)
project(vision_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 添加C++可执行文件
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge OpenCV)

# 安装C++可执行文件
install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

# 安装Python脚本（确保可执行）
install(PROGRAMS
  vision_node/yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 添加 YOLO 视频订阅节点
add_executable(yolo_video_subscriber src/yolo_video_subscriber.cpp)
ament_target_dependencies(yolo_video_subscriber rclcpp sensor_msgs cv_bridge)

# 链接 OpenCV 库
target_link_libraries(yolo_video_subscriber ${OpenCV_LIBRARIES})

install(TARGETS
  yolo_video_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过copyright和cpplint检查（如果需要）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()