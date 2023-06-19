# Limo ROS2 Humble

## 1. Introduction of Function Package

```
└── limo_ros2
    ├── limo_base
    ├── limo_bringup
    ├── limo_car
    ├── limo_msgs
    └── Readme.md

```

limo_base ：This folder is the driver package

limo_bringup：This folder stores some launch files

limo_car：The folder is gazebo simulation function package

limo_msgs：This folder is some message files

## 2. Environment

### Development Environment

 ubuntu 22.04 + [ROS2 Humble desktop full](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

### Download and install required function package

Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```
sudo apt-get install ros-humble-joint-state-publisher-gui 
```

Download and install rqt-robot-steering plug-in, rqt_robot_steering is a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar

```
sudo apt-get install ros-humble-rqt-robot-steering 
```

Download and install teleop-twist-keyboard

###  Download package and Build

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/agilexrobotics/limo_ros2.git
cd catkin_ws
colcon build
```

## Usage

Start the base node for limo

```
ros2 launch limo_base limo_base.launch.py 
```

Start the keyboard teleop node

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# statement

The limo_car gazebo simulation function package is provided by us and the Institute for **Intermodal Transport and Logistics SystemsTU Braunschweig, Germany **jointly developed, thanks for their efforts











