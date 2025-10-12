from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def create_nodes(context):
        world_name_val = LaunchConfiguration('world_name').perform(context)
        stats_rate_val = LaunchConfiguration('stats_rate').perform(context)
        pose_rate_val = LaunchConfiguration('pose_rate').perform(context)
        throttle_path_val = LaunchConfiguration('throttle_path').perform(context)
        ns_val = LaunchConfiguration('ros_namespace').perform(context)

        # Construct input topics based on world name
        gz_stats_src = f'/world/{world_name_val}/stats' if world_name_val else '/stats'
        gz_pose_src = f'/world/{world_name_val}/pose/info' if world_name_val else '/pose/info'

        nodes = []

        # Start throttles
        nodes.append(ExecuteProcess(
            cmd=[
                throttle_path_val, '--type', 'world_stats',
                gz_stats_src, '/throttled/stats', stats_rate_val
            ],
            output='screen',
            name='gz_stats_throttler',
            shell=True
        ))
        
        nodes.append(ExecuteProcess(
            cmd=[
                throttle_path_val, '--type', 'pose_v',
                gz_pose_src, '/throttled/pose/info', pose_rate_val
            ],
            output='screen',
            name='gz_pose_throttler',
            shell=True
        ))

        # JSON stats bridge
        nodes.append(Node(
            package='gz_physics_bridge',
            executable='stats_bridge',
            namespace=ns_val,
            name='stats_bridge',
            output='screen',
            parameters=[{
                'gz_input': '/throttled/stats',
                'ros_output': f'/world/{world_name_val}/stats_json' if world_name_val else '/stats_json'
            }]
        ))

        # JSON pose info bridge
        nodes.append(Node(
            package='gz_physics_bridge',
            executable='pose_info_json_bridge',
            namespace=ns_val,
            name='pose_info_json_bridge',
            output='screen',
            parameters=[{
                'gz_input': '/throttled/pose/info',
                'ros_output': f'/world/{world_name_val}/pose/info_json' if world_name_val else '/pose/info_json'
            }]
        ))

        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='empty', description='World name for Gazebo topics'),
        DeclareLaunchArgument('stats_rate', default_value='1', description='Stats throttling rate in Hz'),
        DeclareLaunchArgument('pose_rate', default_value='1', description='Pose throttling rate in Hz'),
        DeclareLaunchArgument('throttle_path', default_value='/project-root/ros2_ws/src/gz_throttle_tool/build/gz_throttle', description='Path to throttle executable'),
        DeclareLaunchArgument('ros_namespace', default_value='', description='ROS namespace for bridge nodes'),
        OpaqueFunction(function=create_nodes),
    ])


