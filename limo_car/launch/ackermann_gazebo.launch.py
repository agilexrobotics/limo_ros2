# Autor: Zhui Li
# E-Mail: lz554113510@gmail.com
# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # definiert Path für Modell
    package_name = 'limo_car'
    world_file_path = 'worlds/empty_world.model'
    rviz_path = 'rviz/gazebo.rviz'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)
    default_rviz_config_path = os.path.join(pkg_path, rviz_path)

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Position dafür, wo die Modelle herstellt werden
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    mbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch', 'ackermann.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'world': world_path}.items()
    )

    # Einbindung der Gazebo-Startdatei, die im gazebo_ros-Paket enthalten ist
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # laufen ein leere node aus den gazebo_ros package
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mbot',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val],
                        output='screen')


    return LaunchDescription([
        mbot,
        gazebo,
        spawn_entity,
        rviz_arg,
        rviz_node
    ])