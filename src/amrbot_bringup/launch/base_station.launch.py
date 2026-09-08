from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    control_pkg = FindPackageShare('amrbot_control').find('amrbot_control')

    teleop_joy_launch = PathJoinSubstitution([
        control_pkg,
        'launch',
        'teleop_joy.launch.py'
    ])

    include_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_joy_launch)
    )

    gui_node = Node(
        package='amrbot_control',
        executable='gui_node',
        name='gui_node',
        output='screen'
    )

    return LaunchDescription([
        include_teleop_joy,
        gui_node,
    ])