import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='repeat_rate',
            default_value='50.0'
        ),
        launch_ros.actions.Node(
            package='teleop_twist_keyboard',
            node_executable='teleop_twist_keyboard',
            name='teleop_twist',
            # parameters=[
            #     {
            #         'repeat_rate': launch.substitutions.LaunchConfiguration('repeat_rate')
            #     }
            # ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
