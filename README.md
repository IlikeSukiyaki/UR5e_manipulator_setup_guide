# UR5e_manipulator_setup_guide
This is a fast setup guide for my own usage as a reference to set up UR5e manipulator under ubuntu 20.04 and ROS1. No guarantees on its generalization and effectiveness on other settings
# Real-robot Setup with UR5e on Ubuntu 20.04 and ROS Noetic

## Prerequisites

- **Operating System:** Ubuntu 20.04
- **ROS Version:** Noetic
- **Manipulator:** UR5e

## Installation Steps

### 1. Install the Universal Robots ROS Driver

For detailed instructions, refer to the official repository:  
[Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

**Quick Setup Guide:**

```bash
# 1. Source your ROS environment
source /opt/ros/noetic/setup.bash

# 2. Create a workspace and navigate into it
mkdir -p UR_ws/src && cd UR_ws

# 3. Clone the Universal Robots ROS Driver
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# For users in mainland China (domestic acceleration):
# git clone https://ghp.ci/https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# 4. Clone the `universal_robot` description package using the melodic-devel branch
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot
# For users in mainland China (domestic acceleration):
git clone -b melodic-devel https://ghp.ci/https://github.com/ros-industrial/universal_robot.git src/universal_robot
```
### 2. Install Dependency
```bash
# Install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the workspace
catkin_make

# Activate the workspace (source it)
source devel/setup.bash
```
After completing these steps, you can proceed with setting up the robot’s IP configuration and following the instructions in the driver documentation to connect and control the UR5e manipulator.

# Understanding the Installed Files

After completing the driver installation, you will find two folders in the workspace's `src` directory:

1. **`universal_robot`**  
   - This folder contains:
     - **UR robot description files**: Defines the UR robot's kinematic and dynamic model.  
     - **MoveIt! packages**: Provides motion planning capabilities for the UR robot.  

2. **`Universal_Robots_ROS_Driver`**  
   - This folder includes the **driver** that connects the UR robot to the ROS environment.

---

**Key Points**:  
- `universal_robot` → Robot description and MoveIt! support.  
- `Universal_Robots_ROS_Driver` → ROS driver for UR robots.  

Make sure both folders exist in the `src` directory to ensure correct setup and functionality.
### Hello_world of UR5e
![Demonstration of UR5e Setup](./assets/start_demo.gif)
