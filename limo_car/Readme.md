# Limo Simulation Operation Process

## 1.	Introduction of Function Package

```
├── limo_car
 ├── gazebo
 ├── launch
 ├── log
 ├── meshes
 ├── rviz
 ├── src
 ├── urdf
 ├── worlds
```

	gazebo/urdf: The file is used to store the models in Gazebo/Rviz.
	
	launch: This file contains all the files used for launching.
	
	meshes: This file contains the 3D-files for model building.
	
	rviz/worlds: This file is the environment data loaded by the model in the Rviz/Gazebo.

## 2.	Environment

### Development Environment

	ubuntu 22.04 LTS + [ROS Humble desktop full](http://wiki.ros.org/humble/Installation/Ubuntu)

### Download and install required function package


	Download and install all gazebo function package
```
sudo apt install ros-humble-gazebo-*
```

	Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```
sudo apt-get install ros-humble-joint-state-publisher-gui 
```

	Download and install rqt-robot-steering plug-in, rqt_robot_steering is a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar

```
sudo apt-get install ros-humble-rqt-robot-steering 
```



## 3.	About Usage


### 1 Copy the file directly and run it under ~/limo after the start of the second item in 2


### 1.1 Or create workspace, copy simulation model function package and compile

		Open a new terminal and create a workspace named limo_ws, enter in the terminal:

```
mkdir limo_ws
```

		Enter the limo_ws folder

```
cd limo_ws
```

		Create a folder to store function package named src
```
mkdir src
```

		Enter the src folder

```
cd src
```

		Create a package with cmake named limo_car

```
ros2 pkg create --build-type ament_cmake limo_car
```

		Initialize folder

```
cd ..
colcon build
```

		Replace the limo_car created in the file with the limo_car in limo（copy）
		Place files "gazebo, launch, rviz, urdf, meshes, worlds" in ~/limo_ws/install/limo_car/share/limo_car		



### 2.	Run the star file of limo model and visualize the urdf file in Rviz

	Enter the limo_ws folder, if you used usage in 1.1

```
cd limo_ws
```

	Declare the environment variable

```
. install/setup.bash
```

	Run the start file of limo and visualize the model in Rviz

```
ros2 launch limo_car display_ackermann.launch.py 
```


### 3.	Start the gazebo simulation environment of limo and control limo movement in the gazebo

	Enter the limo_ws folder, if you used usage in 1.1

```
cd limo_ws
```

	Declare the environment variable

```
. install/setup.bash
```

	Start the simulation environment of limo, limo have two movement mode, the movement mode is Ackermann mode

```
ros2 launch limo_car ackermann_gazebo.launch.py
```

	Start rqt_robot_steering movement control plug-in, the sliding bar can control the robot motion

```
ros2 run rqt_robot_steering rqt_robot_steering
```


Author: Zhui Li
email: lz554113510@gmail.com
Institute for Intermodal Transport and Logistics Systems
TU Braunschweig
Germany 





