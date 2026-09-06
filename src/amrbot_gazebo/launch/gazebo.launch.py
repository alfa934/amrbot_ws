import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():

    description_pkg = get_package_share_directory('amrbot_description')
    gazebo_pkg = get_package_share_directory('amrbot_gazebo')
    
    gz_resource_path = os.path.dirname(description_pkg)
    
    default_model_path = os.path.join(description_pkg, 'urdf', 'amrbot_description.urdf.xacro')
    default_rviz_path = os.path.join(description_pkg, 'rviz', 'config.rviz')

    world_path = os.path.join(gazebo_pkg, 'worlds', 'my_world.sdf')
    bridge_config_path = os.path.join(gazebo_pkg, 'config', 'gz_bridge_config.yaml')

    ekf_config_path = os.path.join(gazebo_pkg, 'config', 'ekf.yaml')


    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'
    )
    declare_model = DeclareLaunchArgument(
        name='model', default_value=default_model_path, description='Absolute path to robot model file'
    )
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='true', description='Start RViz'
    )
    declare_rviz_config = DeclareLaunchArgument(
        name='rvizconfig', default_value=default_rviz_path, description='Absolute path to rviz config file'
    )


    set_gz_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)
    start_gz_server = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')


    robot_description = Command(['xacro ', LaunchConfiguration('model'), ' sim:=true'])


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_spawn_model.launch.py'
            )
        ),
        launch_arguments={
            'topic': '/robot_description',
            'entity_name': 'amrbot',
            'z': '1.0',
        }.items(),
    )

    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
        ekf_config_path,
        {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ],
    remappings=[('odometry/filtered', 'odometry/ekf')]   
)


    return LaunchDescription([
        set_gz_resource,
        declare_use_sim_time,
        declare_model,
        declare_rviz,
        declare_rviz_config,
        start_gz_server,
        robot_state_publisher_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        ekf_node,
    ])