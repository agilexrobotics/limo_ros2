import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='port_name',
            default_value='ttyTHS1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='odom_topic_name',
            default_value='odom'
        ),
        launch_ros.actions.Node(
            package='limo_base',
            node_executable='limo_base',
            name='limo_base_node',
            output='screen',
            parameters=[
                {
                    'port_name': launch.substitutions.LaunchConfiguration('port_name')
                },
                {
                    'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
