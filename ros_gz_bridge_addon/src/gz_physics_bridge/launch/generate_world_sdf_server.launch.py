from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    world = LaunchConfiguration('world', default='empty')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty'),
        Node(
            package='gz_physics_bridge',
            executable='generate_world_sdf_server',
            name='generate_world_sdf_server',
            parameters=[{'world': world}],
            output='screen'
        )
    ])


