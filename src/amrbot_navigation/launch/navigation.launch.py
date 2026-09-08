from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    navigation_pkg = FindPackageShare('amrbot_navigation').find('amrbot_navigation')

    params_file = PathJoinSubstitution([
        navigation_pkg,
        'config',
        'navigation_params.yaml'
    ])

    map_file = PathJoinSubstitution([
        navigation_pkg,
        'maps',
        'my_map.yaml'
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the navigation stack'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel')],
        output='screen'
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
                'collision_monitor',
                'waypoint_follower'
            ]
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        smoother_server_node,
        behavior_server_node,
        bt_navigator_node,
        velocity_smoother_node,
        collision_monitor_node,
        waypoint_follower_node,
        lifecycle_manager_node,
    ])