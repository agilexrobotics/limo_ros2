import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyUSB1',
                                         description='usb bus name, e.g. ttyUSB1')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')
    #sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
    #                                             description='Simulation control loop update rate')
    # pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
    #                                              description='Odometry TF')
    
    port_name_arg = LaunchConfiguration('port_name',default='ttylimo')
    odom_frame_arg = LaunchConfiguration('odom_frame',default='odom')
    base_link_frame_arg = LaunchConfiguration('base_frame',default='base_link')
    pub_odom_tf_arg_ = LaunchConfiguration('pub_odom_tf', default='true')

    remapping = [
                ('odom', '/wheel/odom'),
    ]
    limo_base_node = Node(
        package='limo_base',
        executable='limo_base',  #foxy executable='limo_base',
        output='screen',
        name='limo_base_node',
        emulate_tty=True,
        parameters=[{
                # 'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'port_name': port_name_arg,                
                'odom_frame': odom_frame_arg,
                'base_frame': base_link_frame_arg,
                'pub_odom_tf': pub_odom_tf_arg_,
                'use_mcnamu': False
        }],
        remappings=remapping
        )

    return LaunchDescription([
        #DeclareLaunchArgument('pub_odom_tf',default_value=pub_odom_tf_arg_,description='TF'),
        limo_base_node

    ])

