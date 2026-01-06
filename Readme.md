# Limo ROS2 Jazzy

## 1. Introduction of Function Package

```
└── limo_ros2
    ├── limo_base
    ├── limo_bringup
    ├── limo_car
    ├── limo_description
    ├── limo_msgs
    └── Readme.md

```

limo_base ：This folder is the driver package

limo_bringup：This folder stores some launch files and ros-gazebo bridge config

limo_car：The folder is gazebo simulation function package

limo_description: Contains robot description files and rviz config

limo_msgs：This folder is some message files

## 2. Environment

### Development Environment

 ubuntu 24.04 + [ROS2 Jazzy desktop full](https://docs.ros.org/en/jazzy/Installation.html)

### Download and install required function package

Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```
sudo apt-get install ros-humble-joint-state-publisher-gui 
```

Download and install teleop-twist-keyboard

###  Download package and Build

```
mkdir -p limo_ros2_ws/src
cd limo_ros2_ws/src
git clone https://github.com/anshikasinha8/limo_ros2.git
cd limo_ros2_ws
colcon build
source install/setup.bash
```

## Usage

Start the launch file for limo

```
ros2 launch limo_bringup limo_gazebo.launch.xml 
```

Start the keyboard teleop node

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Statement

The limo_car gazebo simulation function package is provided by agilexrobotics and the Institute for **Intermodal Transport and Logistics SystemsTU Braunschweig, Germany **jointly developed, thanks for their efforts

# Author

Name: Anshika Sinha

Email: anshikasinha218@gmail.com 










