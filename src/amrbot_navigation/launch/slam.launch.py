from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    navigation_pkg = FindPackageShare('amrbot_navigation').find('amrbot_navigation')
    slam_params_file = PathJoinSubstitution([navigation_pkg, 'config', 'slam_params.yaml'])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    configure_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )

    activate_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    delayed_configure = TimerAction(period=2.0, actions=[configure_cmd])
    delayed_activate = TimerAction(period=3.0, actions=[activate_cmd])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('amrbot_navigation').find('amrbot_navigation'),
            'rviz',
            'mapping.rviz'
        ])]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        slam_toolbox_node,
        delayed_configure,
        delayed_activate,
        rviz_node,
    ])