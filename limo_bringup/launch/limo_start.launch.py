import os
import sys

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    port_name = DeclareLaunchArgument(name='port_name',
                                             default_value='ttyTHS1')
    odom_topic_name = DeclareLaunchArgument(name='odom_topic_name',
                                             default_value='odom')
    open_rviz = DeclareLaunchArgument(name='open_rviz',
                                             default_value='false')
    
    rviz_node = Node(
        package='rviz2',
        name='rviz2',
        executable='rviz',
        on_exit='kill',
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('open_rviz'))
    )



    static_transform_publisher_node = Node(
        package='tf2_ros',
        name='static_transform_publisher',
        executable='static_transform_publisher',
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "/base_link", "/imu_link"]
    )

    limo_base_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('limo_base'),
                         'launch/limo_base.launch.py')),
        launch_arguments={
            'port_name': launch.substitutions.LaunchConfiguration('port_name'),
            'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name')
        }.items()
    )

    open_ydlidar_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('limo_base'),
                         'launch', 'open_ydlidar_launch.py'))
    )

    ld = LaunchDescription([
        port_name,
        odom_topic_name,
        open_rviz,
        rviz_node,
        static_transform_publisher_node,
        limo_base_launch,
        open_ydlidar_launch
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
