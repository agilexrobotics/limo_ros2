import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='map_file',
            default_value=get_package_share_directory(
                'limon_bringup') + '/maps/limon.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='no_static_map',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='base_global_planner',
            default_value='navfn/NavfnROS'
        ),
        launch.actions.DeclareLaunchArgument(
            name='base_local_planner',
            default_value='base_local_planner/TrajectoryPlannerROS'
        ),
        launch_ros.actions.Node(
            package='amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {
                    'initial_pose_x': '0'
                },
                {
                    'initial_pose_y': '0'
                },
                {
                    'initial_pose_a': '0'
                },
                get_package_share_directory(
                    'limon_bringup') + '/param/amcl_params.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='move_base',
            executable='move_base',
            name='move_base',
            output='screen',
            parameters=[
                {
                    'base_global_planner': launch.substitutions.LaunchConfiguration('base_global_planner')
                },
                {
                    'base_local_planner': launch.substitutions.LaunchConfiguration('base_local_planner')
                },
                {
                    'local_costmap/width': '2.0'
                },
                {
                    'local_costmap/height': '2.0'
                },
                {
                    'global_costmap/width': '2.0'
                },
                {
                    'global_costmap/height': '2.0'
                },
                {
                    'controller_frequency': '50.0'
                },
                get_package_share_directory(
                    'limon_bringup') + '/param/planner.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='map_server',
            executable='map_server',
            name='map_server'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
