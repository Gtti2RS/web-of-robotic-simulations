from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_process_supervisor',
            executable='supervisor',
            name='process_supervisor',
            output='screen',
            parameters=[{
                'grace_timeout_sec': 5,
                'log_child_output': True
            }]
        )
    ])
