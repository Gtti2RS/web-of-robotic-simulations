# Half-Containerized Deployment

For users who already have Gazebo and CoppeliaSim installed on their host system and want to use only the WoT-Server container, you can run a half-containerized setup.

## Prerequisites

**Host System Requirements:**
- Ubuntu 24.04 (or compatible Linux distribution)
- ROS 2 Jazzy Jalisco installed
- Gazebo Harmonic installed
- CoppeliaSim 4.10.0 installed
- Node.js 18+ installed

**Required ROS 2 Packages:**
```bash
# Core ROS 2 packages
sudo apt install ros-jazzy-ros-base

# Gazebo-specific packages
sudo apt install \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-moveit-msgs \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros2controlcli \
  ros-jazzy-trajectory-msgs \
  ros-jazzy-ur \
  ros-jazzy-web-video-server

# CoppeliaSim-specific packages
sudo apt install \
  ros-jazzy-moveit \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-ros-planning \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-robot-state-publisher

# Build custom ROS 2 packages
cd ~/wos/ros_gz_bridge_addon
colcon build --packages-select \
  gz_physics_bridge_interfaces \
  sim_process_supervisor_interfaces \
  gz_physics_bridge \
  sim_process_supervisor \
  ros_gz_bridge_addon \
  gz_throttle_tool
source install/setup.bash
```

## Setup Instructions

**1. Start Only WoT-Server Container:**
```bash
cd ~/wos/docker
docker compose -f compose.dev.yaml up -d WoT_Server
```

**2. Start Gazebo Manually:**
```bash
# Terminal 1: Start Gazebo with process supervisor
source /opt/ros/jazzy/setup.bash
source ~/wos/ros_gz_bridge_addon/install/setup.bash
cd ~/wos/main
ros2 launch sim_process_supervisor supervisor.launch.py
```

**3. Start CoppeliaSim Manually:**
```bash
# Terminal 2: Start CoppeliaSim with ROS 2 helper
cd /path/to/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/
./coppeliaSim -H -GvisualizationStream.autoStart=true -a ~/wos/library/coppeliasim/addOn/ros2_helper.lua
```

**4. Start WoT Controllers:**
```bash
# Terminal 3: Start Gazebo controller
docker exec -it WoT_Server bash
cd /project-root/main
node gz_controller.js

# Terminal 4: Start CoppeliaSim controller  
docker exec -it WoT_Server bash
cd /project-root/main
node cs_controller.js
```

## Required Manual Processes

**For Gazebo:**
- **Process Supervisor**: `ros2 launch sim_process_supervisor supervisor.launch.py`
  - Manages Gazebo simulation lifecycle
  - Provides bridge services for entity management
  - Handles world loading and simulation control

**For CoppeliaSim:**
- **CoppeliaSim with ROS 2 Helper**: `./coppeliaSim -a ros2_helper.lua`
  - Provides ROS 2 services under `/coppeliasim/` namespace
  - Enables simulation control, scene/model management
  - Publishes simulation statistics and model data

**For UR10 Robot (when needed):**
- **MoveIt Stack**: `~/wos/library/coppeliasim/start_moveit.sh`
  - Robot state publisher
  - Joint state bridge (CoppeliaSim → ROS 2)
  - MoveIt move_group for motion planning

## Network Configuration

**ROS 2 Domain ID:**
```bash
# Set consistent domain ID across all processes
export ROS_DOMAIN_ID=21
```

**Port Mapping:**
- WoT Controllers: 8080 (Gazebo), 8081 (CoppeliaSim)
- UR10 Servers: 8083 (Gazebo), 8084 (CoppeliaSim)
- CoppeliaSim Remote API: 23020

## Troubleshooting

**ROS 2 Discovery Issues:**
```bash
# Check if processes can discover each other
ros2 node list
ros2 service list
ros2 topic list
```

**Missing Services:**
- Ensure process supervisor is running for Gazebo
- Verify CoppeliaSim ROS 2 helper is loaded
- Check that custom ROS 2 packages are built and sourced

**Container Communication:**
```bash
# Test container can reach host services
docker exec -it WoT_Server bash
ros2 node list  # Should see host ROS 2 nodes
```

## Process Dependencies

### Gazebo Controller Dependencies
The `gz_controller.js` requires these ROS 2 services to be available:
- `/world/{world_name}/create` - Entity spawning
- `/world/{world_name}/remove` - Entity deletion  
- `/world/{world_name}/set_pose` - Pose setting
- `/world/{world_name}/control` - Simulation control
- `/set_rtf` - Real-time factor control
- `/generate_sdf_file` - SDF file generation

These are provided by the **process supervisor** and **ros_gz_bridge_addon** packages.

### CoppeliaSim Controller Dependencies
The `cs_controller.js` requires these ROS 2 services under `/coppeliasim/` namespace:
- `/coppeliasim/start` - Start simulation
- `/coppeliasim/pause` - Pause simulation
- `/coppeliasim/stop` - Stop simulation
- `/coppeliasim/faster` - Increase simulation speed
- `/coppeliasim/slower` - Decrease simulation speed
- `/coppeliasim/manageScene` - Scene management
- `/coppeliasim/manageModel` - Model management

These are provided by the **CoppeliaSim ROS 2 helper** addOn.

### UR10 Robot Dependencies
When using UR10 robots, additional services are required:
- `/compute_ik` - Inverse kinematics
- `/plan_kinematic_path` - Motion planning
- `/compute_fk` - Forward kinematics
- `/coppelia/ur10/joint_states` - Joint state data

These are provided by the **MoveIt stack** and **joint state bridge**.

## Alternative: Host-Only Deployment

If you prefer to run everything on the host without any containers:

1. Install Node.js dependencies: `npm install`
2. Build ROS 2 packages as described in Prerequisites
3. Start all processes manually on the host system
4. Run controllers directly: `node gz_controller.js` and `node cs_controller.js`

This approach eliminates container networking complexity but requires all dependencies to be installed on the host system.
