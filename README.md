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

## Install URCap on the Physical UR Robot

**URCap** is a plugin that allows the UR robot to be controlled by external devices. Without installing URCap, the robot can only be controlled using the teach pendant.

### Steps to Install URCap:

1. Download the URCap file from the following link:  
   [URCap Installation Guide](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)

2. Follow the instructions in the guide to install URCap on your physical UR robot.

---

For more details and additional setup explanations, refer to this article:  
[Real Robot Setup with UR5e](https://blog.csdn.net/Time_Memory_cici/article/details/130706760)


### Hello_world of UR5e
![Demonstration of UR5e Setup](./assets/start_demo.gif)



### Installation and Testing of ROBOTIQ 2F Grippers
## Proof-of-concept Testing
It is always good to start with a empty workspace and test the component individually. So, in this part, I will walk you through the guide.
## Step 1: Create Test Workspace
```bash
   mkdir -p gripper_ws/src && cd gripper_ws
   cd src
   catkin_init_workspace
```


## Step 2: Clone the Repository

Since the [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq) repository does not contain the Noetic version, we will clone the repository from [jr-robotics/robotiq](https://github.com/jr-robotics/robotiq) instead.

Run the following command to clone the repository:

```bash
git clone https://github.com/jr-robotics/robotiq
```

## Step 3: Install Dependency
```bash
rosdep update

rosdep install --from-paths src --ignore-src -y

cd grippers_ws
catkin_make
source devel/setup.bash
```
## Step4: Running and Debugging the Robotiq Node

This guide explains how to run the Robotiq gripper node, check the serial device information, and troubleshoot any connection issues.

Check Serial Device Information

To check the available serial devices, run the following command:

```bash
dmesg | grep ttyS*
```
This command will list all the available serial ports. If there are multiple devices, you will need to try each one to identify the correct serial port.

### Set Permissions for the Serial Port
Grant all permissions to the identified serial port (e.g., /dev/ttyS0). If there are multiple devices, you may need to try each one.
Run the following command:
```bash
sudo chmod 777 /dev/ttyUSB0
```
**_Once the connection is successful, the blue LED on the Robotiq gripper should light up, indicating that the gripper is ready to be controlled._**
### Run the Gripper Node

```bash
roscore
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```
### Run the control example
The following command gives a control example of the gripper. When the instruction appears, please first press **`r`** to reset and press **`a`** to activate in order to activate the gripper.
```bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
```



For detailed instructions, refer to the following link:  
[Setup Guide for Robotiq Gripper](https://blog.csdn.net/mc17852636978/article/details/129228971)





