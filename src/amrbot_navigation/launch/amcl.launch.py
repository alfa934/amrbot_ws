from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    navigation_pkg = FindPackageShare('amrbot_navigation').find('amrbot_navigation')

    map_file = PathJoinSubstitution([
        navigation_pkg,
        'maps',
        'my_map.yaml'
    ])

    amcl_config = PathJoinSubstitution([
        navigation_pkg,
        'config',
        'amcl_params.yaml'
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            amcl_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/odom', '/odometry/filtered'),
            ('/scan', '/scan'),
        ],
        output='screen'
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])