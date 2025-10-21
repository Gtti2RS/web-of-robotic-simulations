# Native API and ROS2 Calls

This file contains native API calls and ROS2 commands used during development. These commands are useful for debugging, testing, and further development of the simulation environment.

## Gazebo

Message type definitions: https://github.com/gazebosim/gz-msgs/tree/main/proto/gz/msgs

### Start Gazebo
```bash
gz sim empty.sdf (-s --headless-rendering)
```

### Basic Discovery Commands
```bash
# List all available topics
gz topic -l
# Show topic info
gz topic -i -t /topic
# Echo topic 
gz topic -e -t /topic

# List all available services
gz service -l
# Show service info
gz service -i -s /service
```

### Entity Management
```bash
# Create a camera entity
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/camera.sdf", name: "cam", pose: {position: {x: -5, y: 0, z: 5}}'

# Create a cardboard box model (from ros_gz_sim_demos)
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/opt/ros/jazzy/share/ros_gz_sim_demos/models/cardboard_box/model.sdf", name: "cardboard_box", pose: {position: {x: 2.0, y: 2.0, z: 1.0}, orientation: {x:0,y:0,z:0,w:0}}'

# Set entity pose
gz service -s /world/empty/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req 'name: "bot", position: { x: -1.0, y: 2.0, z: 1.0 }'

# Remove entity
gz service -s /world/empty/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 --req 'name: "box", type: 2'
```

### Scene Information
```bash
# Get scene information
gz service -s /world/empty/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 1000 --req

# Generate world SDF (save world)
gz service -s /world/empty/generate_world_sdf --reqtype gz.msgs.SdfGeneratorConfig --reptype gz.msgs.StringMsg --timeout 1000 --req
```

### Simulation Control
```bash
# Run simulation to specific time
gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'run_to_sim_time: {sec:65,nsec:0}'

# Reset simulation
gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'reset: {all:true}'
```

### ROS2-Gazebo Bridge
```bash
# Set up parameter bridge for various topics and services
ros2 run ros_gz_bridge parameter_bridge \
  /world/empty/create@ros_gz_interfaces/srv/SpawnEntity \
  /world/empty/remove@ros_gz_interfaces/srv/DeleteEntity \
  /world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose \
  /world/empty/model/cam/link/link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo

# Remove entity via ROS2 service
ros2 service call /world/shapes/remove ros_gz_interfaces/srv/DeleteEntity "{entity: {name: sphere, type: 2}}"

# Start web video server for camera streaming
ros2 run web_video_server web_video_server --ros-args -p port:=8081 -p address:=0.0.0.0 -p server_threads:=4
```

### Process Management
```bash
# Launch process supervisor
ros2 launch sim_process_supervisor supervisor.launch.py

# Execute command in managed process
ros2 service call /process/exec sim_process_supervisor_interfaces/srv/ExecCmd "{cmd: 'ros2 topic list'}"

# Start managed process
ros2 service call /process/managed/start sim_process_supervisor_interfaces/srv/ManagedStart "{name: 'gz1', cmd: 'gz sim shapes.sdf'}"

# List managed processes
ros2 service call /process/managed/list sim_process_supervisor_interfaces/srv/ManagedList "{}"

# Check process status
ros2 service call /process/managed/status sim_process_supervisor_interfaces/srv/ManagedStatus "{name: 'gz1'}"

# Stop managed process
ros2 service call /process/managed/stop sim_process_supervisor_interfaces/srv/ManagedStop "{name: 'gz1'}"
```

### Physics Bridge
```bash
# Note: Source ros_gz_bridge_addon/install/setup.bash for both wot_server and bridge, sdf file must contain physics plugin

# Launch physics bridge
ros2 launch gz_physics_bridge set_physics_server.launch.py world:=empty

# Generate world SDF via physics bridge
ros2 service call /world/shapes/generate_world_sdf gz_physics_bridge/srv/GenerateWorldSdf

# Launch bridge addon with stats and pose publishing
ros2 launch ros_gz_bridge_addon ros_gz_bridge_addon.launch.py world:=empty stats_rate:=1 pose_rate:=1

# Bridge clock topic
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Robot Control
```bash
# List available controllers
ros2 control list_controllers

# Control UR arm with joint trajectory
ros2 action send_goal /ur_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], points: [{positions: [1.57, -1.57, 1.57, -1.57, 0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}'

# Control RG2 gripper
ros2 action send_goal /rg2_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['rg2_finger_joint1','rg2_finger_joint2'],points: [{ positions: [0.30, 0.30], time_from_start: {sec: 1} }, { positions: [0.10, 0.10], time_from_start: {sec: 2} }]}}"

# Compute inverse kinematics
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{ik_request: {group_name: 'ur_manipulator',ik_link_name: 'tool0',pose_stamped: {header: { frame_id: 'world' },pose: {position: { x: -0.5, y: 0.5, z: 0.25 },orientation: { x: 0.0, y: 0.7071, z: 0.0, w: 0.7071 }}}}}"
```

## CoppeliaSim

### Basic Launch and Control
```bash
# Launch CoppeliaSim in headless mode with ROS2 helper
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04
./coppeliaSim -H -GvisualizationStream.autoStart=true -a /project-root/library/coppeliasim/addOn/ros2_helper.lua
```

### Lua API Commands

See https://manual.coppeliarobotics.com/en/apiFunctions.htm

```lua
-- Simulation control
sim.startSimulation()
sim.wait(2)
sim.stopSimulation()

-- Scene management
sim.loadScene('/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/scenes/controlTypeExamples/controlledViaRos2.ttt')

-- Object manipulation
sim.getObject('/UR10', sim.handle_any)
sim.getObjectAlias(handle)
sim.setObjectPose(h, -1, {0.5, 0.0, 0.2, 0.0, 0.0, 0.707, 0.707})

-- Simulation parameters
sim.setInt32Parameter(sim.intparam_speedmodifier, 5)  -- Range: -3(0.1) to 6(64)

-- Object tree operations
sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 2) --get all handles
sim.getObjectsInTree(30, sim.object_joint_type, 0) --get all joint handles of a robot

-- Joint control
sim.setJointTargetPosition(72, -1.57)
sim.setJointMode(16, sim.jointmode_dynamic)

-- Load helper scripts
dofile('/project-root/library/coppeliasim/addOn/ur10_rg2_helper.lua')
```

### ROS2 Service Integration (with ros2_helper.lua)
```bash
# Start CoppeliaSim simulation
ros2 service call /startCoppeliaSim std_srvs/srv/Trigger

# Pause CoppeliaSim simulation
ros2 service call /pauseCoppeliaSim std_srvs/srv/Trigger

# Stop CoppeliaSim simulation
ros2 service call /stopCoppeliaSim std_srvs/srv/Trigger

# Speed up simulation
ros2 service call /fasterCoppeliaSim std_srvs/srv/Trigger
```

### Scene Management via ROS2
```bash
# Load scene
ros2 topic pub /manageScene std_msgs/msg/String '{"data":"{\"operation\":\"load\",\"scenePath\":\"/opt/coppeliasim410/scenes/controlTypeExamples/controlledViaRos2.ttt\"}"}' --once

# Close current scene
ros2 topic pub /manageScene std_msgs/msg/String '{"data":"{\"operation\":\"close\"}"}' --once
```

### Model Management via ROS2
```bash
# Load UR10 model from CoppeliaSim models
ros2 topic pub /manageModel std_msgs/msg/String '{"data":"{\"operation\":\"load\",\"modelPath\":\"/opt/coppeliasim410/models/robots/non-mobile/UR10.ttm\",\"position\":[0.5,0.0,0.2],\"orientation\":[0.0,0.0,0.707,0.707],\"objectName\":\"MyRobot\"}"}' --once

# Load UR10+RG2 model from URDF
ros2 topic pub /manageModel std_msgs/msg/String '{"data":"{\"operation\":\"load\",\"modelPath\":\"/project-root/Assets/urdf/robots/ur10_rg2/ur10_rg2.urdf\",\"objectName\":\"MyBot\"}"}' --once

# Set object pose
ros2 topic pub /manageModel std_msgs/msg/String '{"data":"{\"operation\":\"setPose\",\"handle\":15,\"pose\":[0.5,0.0,0.2,0.0,0.0,0.707,0.707]}"}' --once

# Remove object
ros2 topic pub /manageModel std_msgs/msg/String '{"data":"{\"operation\":\"remove\",\"handle\":15}"}' --once
```

### Robot Trajectory Control
```bash
# Send joint trajectory to UR10+RG2 robot
ros2 topic pub /ur10_rg2/trajectory std_msgs/msg/String "data: '{\"handle\":30,\"positions\":[1.57, -1.57, 1.57, -1.57, 0, 0, 0, 0]}'" --once
```

