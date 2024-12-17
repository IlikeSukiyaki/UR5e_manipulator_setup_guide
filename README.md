# UR5e_manipulator_setup_guide
This is a fast setup guide for my own usage as a reference to set up UR5e manipulator under ubuntu 20.04 and ROS1. No guarantees on its generalization and effectiveness on other settings
# Real-robot Setup with UR5e on Ubuntu 22.04 and ROS Noetic

## Prerequisites

- **Operating System:** Ubuntu 22.04
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