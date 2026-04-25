cmake_minimum_required(VERSION 3.8)
project(vision_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# camera_node 使用 image_transport 发布原始图像
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)

# yolo_video_subscriber 使用 image_transport 订阅
add_executable(yolo_video_subscriber src/yolo_video_subscriber.cpp)
ament_target_dependencies(yolo_video_subscriber
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)

install(TARGETS
  camera_node
  yolo_video_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()