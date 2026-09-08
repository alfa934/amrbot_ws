from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.2,
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'joy_config': '',
                
                'enable_button': 4, # L1 button
                'enable_turbo_button': -1,
                
                'axis_linear.x': 1,      # left stick Y (up/down)
                'axis_angular.yaw': 3,   # right stick X (left/right)
                
                'scale_linear.x': 0.2,
                'scale_angular.yaw': 0.3,
                'scale_linear_turbo.x': 0.15,
                'scale_angular_turbo.yaw': 0.1,
                
                'publish_stamped_twist': True,
                
                'invert_linear': 1,
                'invert_angular': -1,
            }],
            remappings=[
                ('cmd_vel', '/diff_drive_controller/cmd_vel'),
            ],
        ),
    ])