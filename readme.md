
下面的脚本一条一条运行， 注意不能在后台运行

# 导航

```shell
rviz2
## 启动底盘
ros2 launch limo_bringup limo_start.launch.py
sleep 2
## 启动导航
ros2 launch limo_bringup limo_navigation.launch.py
sleep 2
## 启动定位
ros2 launch limo_bringup limo_localization.launch.py
```

# 建图

```shell
rviz2
ros2 launch limo_bringup limo_start.launch.py
ros2 launch build_map_2d revo_build_map_2d.launch.py
#上面三条指令启动之后，用遥控器控制车子行走
```


# 键盘控制

```shell
ros2 launch limo_bringup limo_start.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# 仿真

## 四轮差速模式

rviz2中显示模型

```
ros2 launch limo_description display_models_diff.launch.py 
```

gazebo中显示模型

```
ros2 launch limo_description gazebo_models_diff.launch.py 
```

启动键盘控制节点控制limo

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 阿克曼模式

开发中





