import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
    use_remappings = LaunchConfiguration('use_remappings')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    remappings = [((namespace, '/tf'), '/tf'),
                  ((namespace, '/tf_static'), '/tf_static'),
                  ('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'bt_xml_filename': bt_xml_file,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}
    
    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'param', 'amcl_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),
        
        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'use_lifecycle_mgr', default_value='true',
            description='Whether to launch the lifecycle manager'),

        DeclareLaunchArgument(
            'use_remappings', default_value='false',
            description='Arguments to pass to all nodes launched by the file'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'),

        Node(
            package='nav2_controller',
            node_executable='controller_server',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(use_remappings),
            remappings=remappings),

        Node(
            package='nav2_planner',
            node_executable='planner_server',
            node_name='planner_server',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(use_remappings),
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            node_executable='recoveries_server',
            node_name='recoveries_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            use_remappings=IfCondition(use_remappings),
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            node_name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(use_remappings),
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            node_executable='waypoint_follower',
            node_name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(use_remappings),
            remappings=remappings),
        
        Node(
            condition=IfCondition(use_lifecycle_mgr),
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'waypoint_follower']}]),
    ])