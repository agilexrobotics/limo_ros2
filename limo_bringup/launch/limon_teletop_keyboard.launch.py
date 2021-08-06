import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='repeat_rate',
            default_value='50.0'
        ),
        launch_ros.actions.Node(
            package='teleop_twist_keyboard',
            node_executable='teleop_twist_keyboard.py',
            name='teleop_keybord',
            output='screen',
            parameters=[
                {
                    'repeat_rate': launch.substitutions.LaunchConfiguration('repeat_rate')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
