from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    world = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World name for Gazebo topics and physics/SDF generation'
    )
    
    stats_rate = DeclareLaunchArgument(
        'stats_rate',
        default_value='1',
        description='Stats throttling rate in Hz'
    )
    
    pose_rate = DeclareLaunchArgument(
        'pose_rate',
        default_value='1',
        description='Pose throttling rate in Hz'
    )
    
    throttle_path = DeclareLaunchArgument(
        'throttle_path',
        default_value='/project-root/ros_gz_bridge_addon/src/gz_throttle_tool/build/gz_throttle',
        description='Path to throttle executable'
    )
    
    ros_namespace = DeclareLaunchArgument(
        'ros_namespace',
        default_value='',
        description='ROS namespace for bridge nodes'
    )
    
    # Get package directories
    gz_physics_bridge_dir = get_package_share_directory('gz_physics_bridge')
    sim_process_supervisor_dir = get_package_share_directory('sim_process_supervisor')
    
    # Include bridge_throttled.launch.py
    bridge_throttled_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gz_physics_bridge_dir,
                'launch',
                'bridge_throttled.launch.py'
            ])
        ]),
        launch_arguments={
            'world_name': LaunchConfiguration('world'),
            'stats_rate': LaunchConfiguration('stats_rate'),
            'pose_rate': LaunchConfiguration('pose_rate'),
            'throttle_path': LaunchConfiguration('throttle_path'),
            'ros_namespace': LaunchConfiguration('ros_namespace')
        }.items()
    )
    
    # Include set_physics_server.launch.py
    set_physics_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gz_physics_bridge_dir,
                'launch',
                'set_physics_server.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )
    
    # Include generate_world_sdf_server.launch.py
    generate_world_sdf_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gz_physics_bridge_dir,
                'launch',
                'generate_world_sdf_server.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )
    
    return LaunchDescription([
        # Declare all arguments
        world,
        stats_rate,
        pose_rate,
        throttle_path,
        ros_namespace,
        
        # Include all launch files
        bridge_throttled_launch,
        set_physics_server_launch,
        generate_world_sdf_server_launch
    ])
