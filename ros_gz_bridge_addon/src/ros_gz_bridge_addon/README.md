# ROS Gazebo Bridge Addon

A ROS2 package that provides a combined launch file for all Gazebo bridge components.

## Description

This package combines multiple launch files into a single convenient entry point:
- `bridge_throttled.launch.py` - Throttled bridge with world-named topics
- `set_physics_server.launch.py` - Physics server configuration
- `generate_world_sdf_server.launch.py` - World SDF generation server
- `supervisor.launch.py` - Process supervisor

## Usage

### Basic Launch (using all defaults)
```bash
ros2 launch ros_gz_bridge_addon ros_gz_bridge_addon.launch.py
```

### Launch with Custom World Name
```bash
ros2 launch ros_gz_bridge_addon ros_gz_bridge_addon.launch.py world:=my_simulation_world
```

### Launch with Custom Rates
```bash
ros2 launch ros_gz_bridge_addon ros_gz_bridge_addon.launch.py world:=test_world stats_rate:=5 pose_rate:=10
```

### Launch with Custom Namespace
```bash
ros2 launch ros_gz_bridge_addon ros_gz_bridge_addon.launch.py world:=robot_world ros_namespace:=/my_robot
```

### Complete Example with All Parameters
```bash
ros2 launch ros_gz_bridge_addon ros_gz_bridge_addon.launch.py \
    world:=warehouse_sim \
    stats_rate:=2 \
    pose_rate:=20 \
    ros_namespace:=/warehouse \
    throttle_path:=/project-root/ros_gz_bridge_addon/src/gz_throttle_tool/build/gz_throttle
```

## Parameters

- `world` (default: 'empty') - World name for Gazebo topics and physics/SDF generation
- `stats_rate` (default: '1') - Stats throttling rate in Hz
- `pose_rate` (default: '1') - Pose throttling rate in Hz
- `throttle_path` (default: '/project-root/ros_gz_bridge_addon/src/gz_throttle_tool/build/gz_throttle') - Path to throttle executable
- `ros_namespace` (default: '') - ROS namespace for bridge nodes

## Dependencies

- `gz_physics_bridge`
- `sim_process_supervisor`
- `launch`
- `launch_ros`

## Building

```bash
cd /project-root/ros_gz_bridge_addon
colcon build --packages-select ros_gz_bridge_addon
source install/setup.bash
```
