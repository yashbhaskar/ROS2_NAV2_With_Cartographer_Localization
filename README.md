# ROS2_NAV2_With_Cartographer_Localization
This ROS 2 repository contains a complete robot simulation setup with URDF, Gazebo sensors, and plugins, enabling autonomous navigation using Nav2 and localization with Cartographer . Includes custom launch files integrating navigation with Nav2, localization with Cartographer, map server, and visualization for real-time autonomous navigation.

---

## Concept of Cartographer Localization

**Cartographer** is a real-time SLAM (Simultaneous Localization and Mapping) framework developed by Google. It uses sensor fusion from LiDAR, IMU, and odometry to build a consistent map and estimate the robot’s pose within that map. During mapping mode, Cartographer incrementally creates a 2D or 3D map while continuously refining it using loop closure detection and pose graph optimization.
In localization mode, instead of creating a new map, Cartographer loads a previously saved map and uses scan matching to align incoming LiDAR scans to the existing map, providing accurate pose estimation even in dynamic or noisy environments. This helps a robot know “where it is” in a known environment and enables reliable autonomous navigation with Nav2.

<img width="1852" height="1050" alt="1" src="https://github.com/user-attachments/assets/7d1a6051-4269-48b5-9e8d-5f8497ba15b8" />

---

## 🚀 Features

- Complete robot simulation with URDF, Gazebo sensors, controllers, and plugins
- Autonomous navigation using Nav2 (global & local planners, costmaps, recovery behaviors)
- Accurate real-time localization using Cartographer SLAM in localization mode
- Integrated launch system including navigation, Cartographer, map server, and RViz
- LiDAR, IMU, and odometry fusion for stable pose estimation
- Works on pre-built map without remapping, enabling repeatable navigation tasks
- RViz visualization with navigation plugins

---

## 🎓 Learning Objectives

- Understand robot modeling with URDF and sensor integration in ROS 2
- Configure and run Nav2 for autonomous navigation and path planning
- Use Cartographer for localization and understand scan matching & pose estimation
- Work with map server and lifecycle nodes in ROS 2 navigation pipeline
- Send and manage navigation goals for autonomous robot movement
- Send navigation goals and observe autonomous movement to target points
- Optimize localization accuracy

---

## Installation

### Make Workspace
```bash
mkdir robot_ws/
```

### Change Workspace
```bash
cd robot_ws
```

### Make src
```bash
mkdir src/
```

### Change Workspace
```bash
cd src
```

### Clone This Repository
```bash
git clone https://github.com/yashbhaskar/ROS2_NAV2_With_Cartographer_Localization.git
```

### Install Nav2
```bash
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### (Optional recommended packages)
```bash
sudo apt install ros-humble-nav2-common
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-behavior-tree
```

### Install Cartographer SLAM
```bash
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
```

### Change Path

- Open the file:
    cartographer_slam/launch/cartographer_localization.launch.py
- In Line no. 22 and 30 Change map and config folder path with your correct path 

### Change Workspace
```bash
cd ..
```

### Build the Package
```bash
colcon build --packages-select my_bot robot_navigation cartographer_slam
source install/setup.bash
```

---

## 🚀 How to Run

### 1st Terminal : Launch robot in gazebo
```bash
ros2 launch my_bot gazebo.launch.py
```

### 2nd Terminal : Start autonoumous navigation
```bash
ros2 launch robot_navigation cartographer_localization.launch.py
```
- Now rviz is open and robot spawn in robot’s initial pose on the map and start Cartographer localization.
- Then use Nav2 Goal / 2D Goal Pose to send a navigation goal. The robot will autonomously plan a path and move toward the target, avoiding obstacles and reaching the goal successfully.
- With Cartographer-based localization, you may still notice a small amount of odometry drift and cases where the inflation layer fails to recognize all obstacles due to imperfections in the pbstream-generated map. Performing precise and consistent mapping directly in Cartographer helps resolve these issues and ensures accurate obstacle representation for reliable localization.

<img width="1852" height="1050" alt="2" src="https://github.com/user-attachments/assets/4ec91f18-7d81-4c50-a07d-136950b4708e" />

---

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com

📌 GitHub: https://github.com/yashbhaskar
