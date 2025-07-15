Demo - https://youtu.be/ennWmgYRBn0


# 🤖 TASK-2: Autonomous Exploration with TurtleBot3 in ROS 2 (Custom Frontier Explorer)

## 🎯 Objective

Enable the **TurtleBot3** robot to autonomously **explore and map an unknown environment** in simulation using **ROS 2**.

---

## ✅ What’s Covered

- 🗺️ Real-time SLAM using LiDAR
- 🔄 Autonomous movement with dynamic goal selection
- ⚙️ Custom exploration logic (`simple_explorer` package)
- 🚧 Obstacle avoidance with Nav2
- 💡 Recovery from navigation failures
- 📡 Rviz2 visualization for robot, map, and goals

---

## 🛠️ Tools and Frameworks

| Tool               | Purpose                                      |
|--------------------|----------------------------------------------|
| **ROS 2 Humble**   | Robotics framework for control and simulation|
| **Gazebo**         | 3D simulation environment                    |
| **Rviz2**          | Visualization of robot, sensors, and map     |
| **TurtleBot3**     | Mobile robot platform (simulated)            |
| **SLAM Toolbox**   | Real-time mapping using LiDAR                |
| **Nav2**           | Navigation stack for path planning           |
| **simple_explorer**| Custom exploration package                   |

---

## 📦 Directory Structure

your_ws/

├── src/

│ ├── simple_explorer/ 

---

## 🔧 Installation & Setup

## 🧰 System Requirements

- Ubuntu 22.04
- ROS 2 Humble installed and sourced

### 1. 📦 Install Required Dependencies

```bash
# ROS 2 development tools
sudo apt update && sudo apt install -y \
    build-essential \
    ros-humble-desktop \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-slam \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-argcomplete \
    git \
    wget \
    curl \
    python3-vcstool \
    ros-humble-tf-transformations
```
### 2. 🧱 Clone and Build Workspace
```bash
cd your_ws
git clone https://github.com/YashSharma-code/task2.git
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```
3. 🌍 Launch Simulation
In terminal 1:

```bash
Launch your world with turtlebot3 robot
Ex:
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
4. 🧠 Launch SLAM Toolbox
In terminal 2:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
5. 🧭 Launch Nav2 (Navigation)
In terminal 3:

```bash
 ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
6. 🖥️ Rviz2 Visualization
In terminal 4:
```bash

 ros2 launch nav2_bringup rviz_launch.py
```
7. 🤖 Launch Custom Autonomous Exploration
In terminal 5:

```bash
source ~/your_ws/install/setup.bash
ros2 run simple_explorer explorer_node
```
This will start your custom frontier exploration node that sends goal positions to the Nav2 stack based on detected unexplored regions in the SLAM map.

🔍 How It Works
SLAM Toolbox builds the environment map in real time using LiDAR.

Nav2 handles local/global path planning and obstacle avoidance.

The simple_explorer node:

Detects frontiers (boundaries between explored and unexplored areas).

Selects the best frontier to explore next.

Sends a goal to the Nav2 NavigateToPose action server.

Waits for result, and repeats until map is complete.


📸 Expected Outcome
The robot autonomously explores the entire map, reaching every frontier until the map is complete — with minimal or no human interaction.
