
This workspace contains three ROS2 packages:
- `bot_description` — URDF + plugins for the robot
- `bot_world` — Custom Gazebo world with launch integration
- `bot_control` — Laser scan reader and field-of-view filter node

---

## ✅ Section 1

### 1. `bot_description` Package

This package contains the URDF/Xacro of the robot designed as per the mechanical diagram from **Bot_1.pdf**.

#### ✅ Features implemented:
- Differential drive with **`libgazebo_ros_diff_drive.so`**
- **LIDAR sensor** with `libgazebo_ros_ray_sensor.so` plugin
- **RGB camera** with `libgazebo_ros_camera.so` plugin
- Joint state publisher GUI + robot_state_publisher
- All launch files required for visualization and control

#### 📂 Key files:
- `urdf/robot.urdf.xacro`
- `launch/rviz.py` — launches robot in RViz
- `launch/spawn.launch.py` — spawns in Gazebo with default world and teleop keyboard integration


---

### 2. `bot_world` Package

This package includes a **custom world** that replicates the structure shown in the question (walls and cones).

#### ✅ Features:
- World designed using Gazebo UI and exported
- Robot spawns **at the center** of the custom world
- Includes launch file to bring up the full simulation + RViz + teleop

#### 📂 Key files:
- `worlds/obstacles.world`
- `launch/world.launch.py`

#### ⚠️ Known Limitation:
- The **robot model currently does not spawn automatically** in the world due to unknown spawn behavior.
- The world loads successfully, and the robot can be added manually using:
  ```bash
  ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot
---

## ✅ Section 2

### 3. `bot_control` Package (In Progress)

This package is used to read and filter LIDAR scan data.

#### 🚧 Implemented so far:
- ROS2 C++ node (`reading_laser.cpp`) that subscribes to `/scan` (LIDAR)
- Reads and filters scan range to 0–120° field of view
- Publishes filtered result to `/filtered_scan`
- Launch file that runs both RViz and this node

#### 📂 Key files:
- `scripts/reading_laser.cpp`
- `launch/control.launch.py`

---

## ✅ How to Test

### Prerequisites
- ROS2 Humble installed
- `xterm`, `gazebo_ros_pkgs`, `teleop_twist_keyboard`, `joint_state_publisher_gui` installed
- Run from a sourced workspace:  
  ```bash
  cd ~/gowtham_ws
  colcon build --symlink-install
  source install/setup.bash
