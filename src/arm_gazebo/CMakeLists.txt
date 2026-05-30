cmake_minimum_required(VERSION 3.8)
project(arm_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    worlds
    config
    launch
    models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
