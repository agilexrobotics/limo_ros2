
Run the following scripts one by one, and do not run them in the background:
 
# Navigation

```shell
rviz2
## Start the chassis
ros2 launch limo_bringup limo_start.launch.py
sleep 2
## Start navigation
ros2 launch limo_bringup limo_navigation.launch.py
sleep 2
## Start localization
ros2 launch limo_bringup limo_localization.launch.py
```

# Mapping

```shell
ros2 launch limo_bringup limo_start.launch.py
ros2 launch build_map_2d revo_build_map_2d.launch.py
# After executing the above three commands, control the car with the remote control to move around and build the map
# Save the built map in limo_bringup/maps directory and compile it with colcon build XXX

```


#　Keyboard Control

```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 launch limo_bringup limo_start.launch.py
```
