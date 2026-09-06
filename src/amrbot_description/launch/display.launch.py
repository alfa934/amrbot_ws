import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    amrbot_description_pkg = get_package_share_directory('amrbot_description')
    
    default_model_path = os.path.join(amrbot_description_pkg, 'urdf', 'amrbot_description.urdf.xacro')
    default_rviz_path = os.path.join(amrbot_description_pkg, 'rviz', 'config.rviz')


    declare_model = DeclareLaunchArgument(
        name='model', default_value=default_model_path, 
        description='Absolute path to robot model file'
    )

    declare_rviz_config = DeclareLaunchArgument(
        name='rvizconfig', default_value=default_rviz_path, 
        description='Absolute path to rviz config file'
    )

    robot_description = Command(['xacro ', LaunchConfiguration('model'), ' sim:=false'])


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        declare_model,
        declare_rviz_config,
        robot_state_publisher_node,
        rviz_node,
    ])