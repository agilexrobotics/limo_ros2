
下面的脚本一条一条运行， 注意不能在后台运行  After installing ROS2 Eloquent [see documents here](docs/README.md) and setting up the software and environment run the following scripts one by one, being careful not to run in the background

# 导航 Navigation

```shell
rviz2
## 启动底盘 start the chassis
ros2 launch limo_bringup limo_start.launch.py
sleep 2

## 启动导航 start navigation
ros2 launch limo_bringup limo_navigation.launch.py
sleep 2

## 启动定位 start positioning
ros2 launch limo_bringup limo_localization.launch.py
```

# 建图 start positioning

```shell
rviz2
ros2 launch limo_bringup limo_start.launch.py
ros2 launch build_map_2d revo_build_map_2d.launch.py
#上面三条指令启动之后，用遥控器控制车子行走 After the above three command are activated use a separate screen to control the car
```


# 键盘控制 keyboard control

```shell
ros2 launch limo_bringup limo_start.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# 仿真 Simulation

## 四轮差速模式  Four wheel differential mode

rviz2中显示模型  Display the model in rviz2

```
ros2 launch limo_description display_models_diff.launch.py 
```

gazebo中显示模型 Run the simulation in gazebo

```
ros2 launch limo_description gazebo_models_diff.launch.py 
```

启动键盘控制节点控制limo Start keyboard teleop to control Limo

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 阿克曼模式  Ackermann Model

开发中  In development





