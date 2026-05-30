cmake_minimum_required(VERSION 3.8)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)

# Install all configuration files and launch files
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
