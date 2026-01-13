# ROS2_NAV2_With_Cartographer_Localization

ROS 2 Nav2 project demonstrating autonomous navigation with Cartographer-based localization on a pre-built map. Includes a complete URDF robot with LiDAR, IMU, and odometry, Gazebo simulation, RViz visualization, and integrated bringup for accurate pose estimation, path planning, and goal execution.

---

## Concept of Cartographer Localization

**Cartographer** is a real-time SLAM (Simultaneous Localization and Mapping) framework developed by Google. It uses sensor fusion from LiDAR, IMU, and odometry to build a consistent map and estimate the robot’s pose within that map. During mapping mode, Cartographer incrementally creates a 2D or 3D map while continuously refining it using loop closure detection and pose graph optimization.
In localization mode, instead of creating a new map, Cartographer loads a previously saved map and uses scan matching to align incoming LiDAR scans to the existing map, providing accurate pose estimation even in dynamic or noisy environments. This helps a robot know “where it is” in a known environment and enables reliable autonomous navigation with Nav2.

<img width="1848" height="1053" alt="Screenshot from 2026-01-13 15-30-42" src="https://github.com/user-attachments/assets/5b6d364f-560a-422b-8586-0a10ec07863d" />

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
- With Cartographer Localization, you will observe highly accurate pose estimation with minimal to no odometry drift, thanks to loop closure and pose graph optimization, which continuously refines the robot’s position on the map for precise localization.

<img width="1848" height="1053" alt="Screenshot from 2026-01-13 15-31-16" src="https://github.com/user-attachments/assets/ec3f8d2a-3dfd-43cb-a8f2-58268bf244e0" />

---

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com

📌 GitHub: https://github.com/yashbhaskar

