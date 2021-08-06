#!/bin/bash

# 启动底盘
ros2 launch limo_bringup limo_start.launch.py &
sleep 2
# 启动导航
ros2 launch limo_bringup limo_navigation.launch.py
sleep 2
# 启动定位
ros2 launch limo_bringup limo_localization.launch.py