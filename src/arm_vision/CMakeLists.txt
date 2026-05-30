cmake_minimum_required(VERSION 3.8)
project(arm_vision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  arm_vision/color_detector.py
  arm_vision/camera_tf_broadcaster.py
  arm_vision/coord_transformer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY arm_vision/
  DESTINATION lib/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
