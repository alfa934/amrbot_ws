from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    localisation_pkg = FindPackageShare('amrbot_localisation').find('amrbot_localisation')
    default_ekf_config = PathJoinSubstitution([localisation_pkg, 'config', 'ekf.yaml'])

    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=default_ekf_config,
        description='Path to EKF configuration file'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
    )

    return LaunchDescription([
        ekf_config_arg,
        ekf_node,
    ])