#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='ur_description').find('ur_description')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    robot_description_file_arg = DeclareLaunchArgument(
        'robot_description_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'urdf', 'ur10_control.urdf'
        ]),
        description='Path to robot description file'
    )
    
    controllers_file_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'ur10_controllers.yaml'
        ]),
        description='Path to controllers configuration file'
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': LaunchConfiguration('robot_description_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            LaunchConfiguration('controllers_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Load and start controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_ur_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'ur_arm_controller'],
        output='screen'
    )
    
    configure_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/controller_manager/configure_controller', 
             'controller_manager_msgs/srv/ConfigureController', 
             '{name: joint_state_broadcaster}'],
        output='screen'
    )
    
    configure_ur_arm_controller = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/controller_manager/configure_controller', 
             'controller_manager_msgs/srv/ConfigureController', 
             '{name: ur_arm_controller}'],
        output='screen'
    )
    
    activate_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'joint_state_broadcaster', 'active'],
        output='screen'
    )
    
    activate_ur_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'ur_arm_controller', 'active'],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_description_file_arg,
        controllers_file_arg,
        robot_state_publisher_node,
        controller_manager_node,
        load_joint_state_broadcaster,
        load_ur_arm_controller,
        configure_joint_state_broadcaster,
        configure_ur_arm_controller,
        activate_joint_state_broadcaster,
        activate_ur_arm_controller,
    ])
