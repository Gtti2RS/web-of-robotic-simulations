# WoRS - Web of Robotic Simulations

A unified Web of Things (WoT) interface for controlling robotic simulations across multiple simulator backends (Gazebo, CoppeliaSim) with ROS 2 integration.

## Features

- **Multi-Simulator Support**: Gazebo Harmonic & CoppeliaSim 4.10
- **WoT Interface**: RESTful API with Thing Descriptions for each simulator
- **ROS 2 Integration**: Jazzy Jalisco with full topic/service support
- **Motion Planning**: MoveIt 2 integration for robotic manipulators
- **Model Management**: URDF/SDF loading, spawning, and manipulation
- **Real-time Monitoring**: Observable properties via Server-Sent Events (SSE)
- **Docker Containerized**: Isolated environments for reproducible simulations

## Architecture

```
┌─────────────┐     ┌──────────────┐     ┌──────────────┐
│  wos_server │────▶│   gz_sim     │     │ coppeliasim  │
│  (Node.js)  │     │  (Gazebo)    │     │  (Sim 4.10)  │
│  WoT Layer  │     │  ROS 2 Node  │     │  ROS 2 Node  │
└─────────────┘     └──────────────┘     └──────────────┘
      │                     │                     │
      └─────────────────────┴─────────────────────┘
                    ROS 2 Network (DDS)
```

### Containers

- **wos_server**: WoT Thing servers exposing simulator controls
- **gz_sim**: Gazebo Harmonic simulator with ROS 2 bridge
- **coppeliasim**: CoppeliaSim 4.10 with custom addOns

## Prerequisites

- Docker & Docker Compose
- Linux (tested on Ubuntu 24.04)
- Ports: 8080-8084, 23000, 23050

## Quick Start

### 1. Build & Start Containers

```bash
docker compose up -d
```

### 2. Start Gazebo Controller

```bash
docker exec -it wos_server bash
cd /project-root/main
node gz_controller.js
```

Access Gazebo WoT interface: `http://localhost:8080/gz_controller`

### 3. Start CoppeliaSim Controller

```bash
docker exec -it wos_server bash
cd /project-root/main
node cs_controller.js
```

Access CoppeliaSim WoT interface: `http://localhost:8081/cs_controller`

### 4. Start CoppeliaSim GUI (Optional)

```bash
docker exec coppeliasim bash -c "cd /opt/coppeliasim410 && ./coppeliaSim.sh"
```

## Usage Examples

### Gazebo Simulation

```javascript
// Load a world
PUT http://localhost:8080/gz_controller/properties/manageScene
{
  "mode": "load",
  "fileName": "empty.sdf"
}

// Spawn a robot model
PUT http://localhost:8080/gz_controller/properties/manageModel
{
  "mode": "load",
  "fileName": "ur10_rg2.urdf",
  "modelName": "robot1",
  "position": [0, 0, 0.5]
}

// Control simulation
POST http://localhost:8080/gz_controller/actions/simControl
{
  "mode": "play"  // or "pause", "stop", "step"
}
```

### CoppeliaSim Simulation

```javascript
// Load a robot
PUT http://localhost:8081/cs_controller/properties/manageModel
{
  "operation": "load",
  "fileName": "ur10_rg2_coppelia.urdf",
  "objectName": "bot",
  "position": [0, 0, 0]
}

// Control simulation
POST http://localhost:8081/cs_controller/actions/simControl
{
  "mode": "run"  // or "pause", "stop"
}
```

### Robot Control (UR10+RG2)

When a UR10+RG2 robot is loaded, a dedicated WoT server spawns:

```javascript
// Move to Cartesian position (uses MoveIt IK)
POST http://localhost:8084/ur10_server/actions/moveToCartesian
{
  "x": 0.5,
  "y": 0.2,
  "z": 0.3,
  "roll": 0,
  "pitch": 1.57,
  "yaw": 0
}

// Move to joint positions
POST http://localhost:8084/ur10_server/actions/moveToJoint
{
  "positions": [0, -1.57, 1.57, -1.57, -1.57, 0]
}

// Gripper control
POST http://localhost:8084/ur10_server/actions/gripClose
POST http://localhost:8084/ur10_server/actions/gripOpen
```

## Project Structure

```
wos/
├── main/                      # WoT Thing server implementations
│   ├── gz_controller.js       # Gazebo controller
│   ├── cs_controller.js       # CoppeliaSim controller
│   └── *.json                 # Thing Descriptions
├── library/                   # Core libraries
│   ├── common/                # Shared utilities
│   │   ├── fileUtils.js       # Asset & file management
│   │   └── ros2_utils.js      # ROS 2 helpers
│   ├── gazebo/                # Gazebo-specific
│   │   ├── gz_ros2_srv.js     # ROS 2 service handlers
│   │   └── gz_bridge_manager.js
│   └── coppeliasim/           # CoppeliaSim-specific
│       ├── cs_action_handlers.js
│       ├── start_moveit.sh    # MoveIt launcher
│       └── addOn/             # CoppeliaSim addOns
├── Assets/                    # Robot models & worlds
│   ├── urdf/                  # URDF robot models
│   ├── gazebo/                # SDF worlds & models
│   └── coppeliasim/           # CoppeliaSim scenes
├── docker/                    # Docker configurations
├── compose.yaml               # Docker Compose setup
└── Dockerfile.*               # Container definitions
```

## Supported Robots

### Pre-configured

- **UR10 + RG2 Gripper**: Universal Robots UR10 with OnRobot RG2 gripper
  - MoveIt motion planning
  - Cartesian & joint control
  - Independent gripper control

### Adding New Robots

1. Place URDF in `Assets/urdf/examples/robots/<robot_name>/`
2. Create MoveIt config (if using motion planning)
3. Update controller detection in `cs_action_handlers.js` or `gz_ros2_srv.js`

## 🔍 Monitoring & Debugging

### Observable Properties (SSE)

Subscribe to real-time updates:

```bash
# Gazebo stats
curl -N http://localhost:8080/gz_controller/properties/simStats

# Model poses
curl -N http://localhost:8080/gz_controller/properties/poses

# CoppeliaSim stats
curl -N http://localhost:8081/cs_controller/properties/simStats
```

### Logs

```bash
# Container logs
docker logs wos_server
docker logs gz_sim
docker logs coppeliasim

# MoveIt logs
docker exec wos_server tail -f /tmp/move_group.log

# Robot state publisher
docker exec wos_server tail -f /tmp/robot_state_publisher.log
```

## 🛠️ Development

### Installing Dependencies

MoveIt packages are required for motion planning:

```bash
# In gz_sim container
docker exec gz_sim apt-get update && apt-get install -y \
  ros-jazzy-moveit ros-jazzy-ur-moveit-config

# In wos_server container (for CoppeliaSim)
docker exec wos_server apt-get update && apt-get install -y \
  ros-jazzy-moveit ros-jazzy-moveit-ros-move-group
```

### Testing

```bash
# Run load tests
cd test
./load_test.sh
```

## Troubleshooting

### MoveIt not starting

```bash
# Check if packages are installed
docker exec wos_server dpkg -l | grep moveit

# Check MoveIt logs
docker exec wos_server tail -50 /tmp/move_group.log
```

### CoppeliaSim connection issues

```bash
# Verify CoppeliaSim is running
docker exec coppeliasim pgrep coppeliaSim

# Check ROS 2 topics
docker exec wos_server bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```

### Path resolution errors

The `fileUtils.js` handles vendor asset paths. For CoppeliaSim vendor assets, "ungrouped" is a virtual folder and not part of the actual filesystem path.

## Configuration

### Environment Variables

- `ROS_DOMAIN_ID=21`: Isolates ROS 2 network
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`: DDS implementation
- `GZ_SIM_RESOURCE_PATH`: Gazebo asset search paths
- `QT_QPA_PLATFORM=offscreen`: Headless rendering

### Network

All containers share the `wos_net` bridge network (172.28.0.0/16) for seamless communication.

## API Documentation

Thing Descriptions (OpenAPI-like) are available at:

- Gazebo: `http://localhost:8080/gz_controller`
- CoppeliaSim: `http://localhost:8081/cs_controller`
- UR10 Server: `http://localhost:8084/ur10_server`

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test in Docker containers
4. Submit a pull request

## License

MIT License

## Authors

Yifan & Cursor & ChatGPT

## Related Projects

- [Web of Things (W3C)](https://www.w3.org/WoT/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/)
- [CoppeliaSim](https://www.coppeliarobotics.com/)
- [MoveIt 2](https://moveit.ros.org/)
