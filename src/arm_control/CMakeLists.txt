cmake_minimum_required(VERSION 3.8)
project(arm_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts and make them executable
install(PROGRAMS
  arm_control/pick_place_node.py
  arm_control/gripper_controller.py
  arm_control/moveit_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the Python package (for imports)
install(
  DIRECTORY arm_control/
  DESTINATION lib/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
