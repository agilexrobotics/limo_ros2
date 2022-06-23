# Limo ROS2 Manual (Translated English)

## Log into Limo
ssh to your Jetson Nano on the Limo

Username `agilex`

Password `agx`

## Install nomachine

`Install nomachine : https://www.nomachine.com/download/download&id=115&s=ARM`

The wired connection address is configured as 192.168.1.3, gateway 192.168.1.1   

## Create workspace
*Note if you are doing this on the existing AgileX image with ros melodic, create a workspace called agilex_ros2_ws - in the Chinese version of the documentation they called their workspace agilex_ws which collides with the Limo melodic workspace*

```
mkdir ~/agilex_ros2_ws
cd ~/agilex_ros2_ws
mkdir src
cd src
git clone --recursive https://github.com/agilexrobotics/limo_ros2.git
cd limo_ros2
rm ydlidar_ros2/params/ydlidar.yaml
ls -s limo_bringup/param/ydlidar.yaml ydlidar_ros2/params/ydlidar.yaml
cd ~/agilex_ros2_ws
```

## Install YDlidarSDK
```
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLIDAR/YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
cd ~/agilex_ros2_ws
```


## Install ROS2 Eloquent

First follow these instructions https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html to install ROS2 Eloquent

Then add colcon and ros2 cartographer
`sudo apt install python3-colcon-core`
`sudo apt install ros-eloquent-cartographer`

## Setup Agilex Workspace

```
cd ~/agilex_ros2_ws
source /opt/ros/eloquent/setup.bash
colcon build
```

## Setup ~/.bashrc for environment configuration

Append the following to your boot

```
## ros2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/eloquent/setup.bash
source /home/agilex/agilex_ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${HOME}/agilex_ws/install/ugv_sdk/lib/ugv_sdk/:${HOME}/agilex_ros2_ws/install/async_port/lib/async_port:/opt/ros/eloquent/lib/:/usr/ local/lib/

echo agx | sudo -S chmod 666 /dev/ttyTHS1
```

## Install RMW 

`sudo apt install ros-eloquent-rmw-fastrtps*`



## Install Nav2 :
`sudo apt install ros2-eloquent-navigation2 ros-eloquent-nav2-amcl ros-eloquent-nav2*`

To launch run `ros2 launch build_map_2d revo_build_map_2d.launch.py`

## Save Map 

`ros2 run nav2_map_server map_saver -f limo`

Change yaml's free_thresh to 0.196

Put the saved pictures limo.pgm, limo.yaml in the ~/agilex_ws/src/drivers/limo_ros2/limo_bringup/maps directory

The image will not take effect until colcon build compiles

## start navigation

`ros2 launch limo_bringup limo_navigation.launch.py`

## Start targeting

`ros2 launch limo_bringup limo_localization.launch.py`

## Steps to start while presenting

Note that in order, localization is placed at the end. 

During the test, it is found that the /map sent by nav2_map_server sometimes cannot keep last, so the start of map_server is placed at the end. 

If there is no map, please try to start limo_localization.launch.py several times to load the map

## rviz2
rviz2
### start the chassis
ros2 launch limo_bringup limo_start.launch.py
sleep 2
### start navigation
ros2 launch limo_bringup limo_navigation.launch.py
sleep 2
### start positioning
ros2 launch limo_bringup limo_localization.launch.py
