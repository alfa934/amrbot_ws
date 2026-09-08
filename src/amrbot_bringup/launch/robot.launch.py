from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    controllers_pkg = FindPackageShare('amrbot_controllers').find('amrbot_controllers')
    spawn_controllers_launch = PathJoinSubstitution([
        controllers_pkg,
        'launch',
        'spawn_controllers.launch.py'
    ])

    localisation_pkg = FindPackageShare('amrbot_localisation').find('amrbot_localisation')
    ekf_launch = PathJoinSubstitution([
        localisation_pkg,
        'launch',
        'ekf.launch.py'
    ])

    manager_node = Node(
        package='amrbot_control',
        executable='manager_node',
        name='manager_node',
        output='screen'
    )

    include_spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_controllers_launch)
    )

    include_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch)
    )

    return LaunchDescription([
        include_spawn_controllers,
        manager_node,
        include_ekf,
    ])