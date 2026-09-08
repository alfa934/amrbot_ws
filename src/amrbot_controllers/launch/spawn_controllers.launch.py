from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    description_pkg = FindPackageShare('amrbot_description').find('amrbot_description')
    controllers_pkg = FindPackageShare('amrbot_controllers').find('amrbot_controllers')

    default_urdf_path = PathJoinSubstitution([
        description_pkg,
        'urdf',
        'amrbot_description.urdf.xacro'
    ])

    default_controller_config = PathJoinSubstitution([
        controllers_pkg,
        'config',
        'controllers.yaml'
    ])

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
        description='Path to the robot URDF/xacro file'
    )

    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=default_controller_config,
        description='Path to the controller configuration YAML file'
    )

    robot_description = {
        'robot_description': Command([
            'xacro ',
            LaunchConfiguration('urdf_path'),
            ' sim:=false'
        ])
    }

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_gui': False, 'source_list': ['/joint_states'], 'rate': 1.0}],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': False}],
        output='screen'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            robot_description,
            LaunchConfiguration('controller_config'),
            {'use_sim_time': False}
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--activate'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager', '--activate'],
        output='screen'
    )

    return LaunchDescription([
        urdf_path_arg,
        controller_config_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])