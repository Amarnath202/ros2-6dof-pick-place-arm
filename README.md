# 🤖 ROS2 6-DOF Pick and Place Arm

## 📌 Overview

A ROS2 Humble based **6-DOF Robotic Arm Simulation** developed using modern robotics tools and frameworks including:

* ROS2 Humble
* Gazebo Classic
* MoveIt2
* ros2_control
* RViz2
* YOLOv8 Vision Pipeline
* EasyOCR
* TF2 Transform System

This project simulates a complete **Pick-and-Place Automation Pipeline** with motion planning, controller integration, workspace simulation, and future vision-based object detection capabilities.

---

# 🚀 Features

## 🦾 Robot Description

* XACRO-based robot model
* 6 Revolute Joints
* Parallel Gripper
* ros2_control Integration
* Gazebo Plugins
* TF Tree Generation

## 🎯 Motion Planning

* MoveIt2 Integration
* Inverse Kinematics (IK)
* Cartesian Path Planning
* Collision Avoidance
* Workspace Constraints

## 🌍 Simulation Environment

* Gazebo Classic Environment
* Worktable Setup
* Pick Objects (Red & Blue Cubes)
* Place Zone
* Physics Tuning
* Controller Tuning

## 👁️ Vision Pipeline

* YOLOv8 Object Detection
* EasyOCR Text Recognition
* Camera TF Transformation
* Object Pose Estimation

## 🤏 Pick & Place Automation

* Topic-Based Trigger
* Automatic Pick Sequence
* Automatic Place Sequence
* Home Position Recovery

---

# 📂 Workspace Structure


arm_ws/
├── src/
│   ├── arm_bringup/
│   ├── arm_control/
│   ├── arm_description/
│   ├── arm_gazebo/
│   ├── arm_moveit_config/
│   └── arm_vision/
├── build/
├── install/
└── log/


---

# ⚙️ Build Instructions

## Clone Repository


git clone https://github.com/Amarnath202/ros2-6dof-pick-place-arm.git

cd ros2-6dof-pick-place-arm


## Build Workspace


cd ~/Projects/arm/arm_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash


---

# ▶️ Launch Complete System


cd ~/Projects/arm/arm_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch arm_bringup bringup.launch.py

### This launches:

✅ Gazebo Simulation
✅ Robot Model
✅ Controllers
✅ MoveIt2
✅ RViz2
✅ Vision Pipeline
✅ Pick & Place Node

---

# 🎮 Manual Pick and Place Trigger

After launching the complete system:


source ~/Projects/arm/arm_ws/install/setup.bash

ros2 topic pub --once /pick_place/trigger \
std_msgs/msg/String "{data: 'start'}"


### Send Robot to Home Position


ros2 topic pub --once /pick_place/trigger \
std_msgs/msg/String "{data: 'home'}"


---

# 🔍 Useful Debug Commands

## Check Running Nodes


ros2 node list


## Check Topics


ros2 topic list

## Check Controllers


ros2 control list_controllers


## Check Joint States


ros2 topic echo /joint_states --once


## Monitor Arm Status


ros2 topic echo /arm/status


## Monitor Pick & Place Trigger


ros2 topic echo /pick_place/trigger


---

# 👀 Vision Testing

## Launch Vision Pipeline Only


ros2 launch arm_vision vision.launch.py


## Monitor Object Detection


ros2 topic echo /detected_object_pose


---

# 📊 Current Development Status

## ✅ Completed

* 6-DOF Robot Arm Model
* Gazebo Simulation Environment
* MoveIt2 Motion Planning
* ros2_control Integration
* RViz Visualization
* Manual Pick-and-Place Trigger
* Controller Tuning
* Physics Optimization

## 🚧 In Progress

* Stable Cube Grasping
* Object Attachment Logic
* Vision-Guided Pick & Place
* YOLO-Based Autonomous Triggering
* Improved Gripper Physics

---


# 🛠️ Technologies Used

| Technology     | Purpose                    |
| -------------- | -------------------------- |
| ROS2 Humble    | Robotics Middleware        |
| Gazebo Classic | Physics Simulation         |
| MoveIt2        | Motion Planning            |
| RViz2          | Visualization              |
| ros2_control   | Hardware Control           |
| YOLOv8         | Object Detection           |
| EasyOCR        | Text Recognition           |
| TF2            | Coordinate Transformations |

---

# 🎯 Future Enhancements

* Autonomous Object Detection
* Multi-Object Sorting
* Dynamic Obstacle Avoidance
* Real Robot Deployment
* AI-Based Grasp Planning
* Vision-Guided Manipulation

---

**Demo Video and Screenshots**
Video:  https://drive.google.com/file/d/15pcy869klUqO2WLYAe_hjW-puLjh-joK/view?usp=sharing
Image 1: https://drive.google.com/file/d/1MKZ30yjYxbI8aT7_V0dzdusUC2QNTliX/view?usp=drive_link
Image 2: https://drive.google.com/file/d/1jLnwR9LPGp2wj7ToAB6-Z_PBMJNKxp1U/view?usp=drive_link
Image 3: https://drive.google.com/file/d/1TKB4nbcMhTHNqV7djAoYaD9hJnfn-KhO/view?usp=drive_link

# 👨‍💻 Author

### Amarnath A

ROS2 • Gazebo • MoveIt2 • Robotics Simulation • Computer Vision

🔗 GitHub: https://github.com/Amarnath202

---

# ⭐ Support

If you found this project useful:

⭐ Star the repository

🍴 Fork the repository

🛠️ Contribute improvements

📢 Share with the robotics community
