import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    description_pkg = get_package_share_directory('amrbot_description')
    controllers_pkg = get_package_share_directory('amrbot_controllers')
    
    default_urdf_path = os.path.join(description_pkg, 'urdf', 'amrbot_description.urdf.xacro')
    
    default_controller_config = os.path.join(controllers_pkg, 'config', 'controllers.yaml')

    robot_description = {'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path'), ' sim:=false'])}

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_gui': False, 'source_list': ['/joint_states']}],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': False}],
        output='screen',
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[robot_description, LaunchConfiguration('controller_config'), {'use_sim_time': False}],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--activate'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager', '--activate'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=default_urdf_path),
        DeclareLaunchArgument('controller_config', default_value=default_controller_config),
        joint_state_publisher_node,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])