# WoRS - Web of Robotic Simulations

A unified Web of Things (WoT) interface for controlling robotic simulations across multiple simulator backends (Gazebo, CoppeliaSim) with ROS 2 integration.

## Features

- **Multi-Simulator Support**: Gazebo Harmonic & CoppeliaSim 4.10
- **WoT Interface**: RESTful API with Thing Descriptions for each simulator
- **ROS 2 Integration**: Jazzy Jalisco with full topic/service support
- **Motion Planning**: MoveIt 2 integration for robotic manipulators
- **Model Management**: URDF/SDF/TTM loading, spawning, and manipulation
- **Real-time Monitoring**: Observable properties via Server-Sent Events (SSE)
- **Docker Containerized**: Isolated environments for reproducible simulations

## Architecture

```
┌─────────────┐     ┌──────────────┐     ┌──────────────────┐
│  WoT_Server │     │  Gazebo_ROS2 │     │ CoppeliaSim_ROS2 │
│             │     │  (Harmonic)  │     │    (4.10.0)      │
│  WoT Layer  │     │  ROS 2 Node  │     │    ROS 2 Node    │
└─────────────┘     └──────────────┘     └──────────────────┘
      │                     │                     │
      └─────────────────────┴─────────────────────┘
                    ROS 2 Network (DDS)
```

### Containers

- **WoT_Server**: WoT Thing servers exposing simulator controls
- **Gazebo_ROS2**: Gazebo Harmonic simulator with ROS 2 bridge
- **CoppeliaSim_ROS2**: CoppeliaSim 4.10 with custom addOns

## Prerequisites

- Docker & Docker Compose
- Linux when deployed directly on host (tested on Ubuntu 24.04)
- Ports: 8080-8084, 8089, 23020

## Quick Start

### 1. Build & Start Containers

### Prerequisites: CoppeliaSim Docker Image

First, build the official CoppeliaSim Docker image:

```bash
# Clone the CoppeliaSim Docker repository
git clone https://github.com/CoppeliaRobotics/docker-image-coppeliasim.git

# Download CoppeliaSim package from https://downloads.coppeliarobotics.com/ and move it to /docker-image-coppeliasim/download/

# Build the CoppeliaSim image
cd docker-image-coppeliasim
docker buildx build --load -t coppeliasim:4.10.0 .
```

### Start the Application

Two compose files are available:

- **`compose.yaml`**: Production mode - automatically starts Gazebo and its WoT controller
- **`compose.dev.yaml`**: Development mode - starts containers and waits for developer to manually start services (allows real-time server log viewing)

```bash
cd docker/

# Production mode (recommended for first-time users)
docker compose up -d

# Development mode (for debugging and development)
docker compose -f compose.dev.yaml up -d
```

### 2. Start Gazebo Controller (dev mode)

1st terminal:
``` bash
docker exec -it Gazebo_ROS2 bash
cd /project-root/main
bash start_process_supervisor.sh
```
2nd terminal:
```bash
docker exec -it WoT-Server bash
cd /project-root/main
node gz_controller.js
```

Thing Description at: `http://localhost:8080/gz_controller` or `http://{server_url}:8080/gz_controller`

### 3. Start CoppeliaSim Controller
Tried to start CoppeliaSim automatically, but can't find a way to overwrite official entrypoint and the container keeps restarting, so currently only dev mode is supported for CoppeliaSim

1st terminal:
```bash
docker exec -it CoppeliaSim_ROS2 bash
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/
./coppeliaSim -H -GvisualizationStream.autoStart=true -a /project-root/library/coppeliasim/addOn/ros2_helper.lua
```
2nd terminal:
```bash
docker exec -it WoT-Server bash
cd /project-root/main
node cs_controller.js
```

Thing Description at: `http://localhost:8081/cs_controller` or `http://{server_url}:8081/cs_controller`

### Alternative: Half-Containerized Deployment

If you already have Gazebo and CoppeliaSim installed on your host system, you can use only the WoT-Server container and run the simulators manually. This approach requires more setup but gives you full control over the simulator processes.

See [HALF_CONTAINERIZED.md](HALF_CONTAINERIZED.md) for detailed instructions.

## Usage Examples

See [test/WoT_API_Examples.md](test/WoT_API_Examples.md)

## Directory Descriptions

**`Assets/`** - Robot models, worlds, and scenes
- **`urdf/`**: URDF robot model definitions with example configurations
- **`gazebo/`**: Gazebo-specific assets including SDF models, worlds, and launch files
- **`coppeliasim/`**: CoppeliaSim-specific assets including models and scene files

**`docker/`** - Docker configurations
- Production and development Docker Compose files for containerized deployment
- Individual Dockerfiles for Gazebo, CoppeliaSim, and WoT server containers
- Entrypoint scripts for container initialization

**`library/`** - Core libraries and utilities
- **`common/`**: Shared utilities for file management, ROS 2 integration, SSE handling, and observable properties
- **`gazebo/`**: Gazebo-specific libraries for ROS 2 service integration, bridge management, and world utilities
- **`coppeliasim/`**: CoppeliaSim-specific libraries including action handlers, MoveIt integration, and custom Lua addOns

**`main/`** - WoT Thing server implementations
- Contains the main WoT controllers (`gz_controller.js`, `cs_controller.js`) that expose simulator controls via RESTful APIs
- Includes Thing Descriptions (JSON files) that define the WoT interface for each simulator
- Features file upload handler and process supervisor for managing simulation processes

**`ros_gz_bridge_addon/`** - Custom ROS 2 workspace
- Contains custom ROS 2 packages for physics bridging, process supervision, and throttling tools
- Includes interface definitions and specialized bridge components for enhanced simulator integration

**`test/`** - Testing utilities and development tools
- Comprehensive testing suite including latency tests, load tests, and bandwidth monitoring
- Development tools: Postman collections, interactive API client, and browser bookmarks
- API documentation and examples for WoT endpoints

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

## Monitoring & Debugging

### Observable Properties (SSE)

Subscribe to real-time updates in web browsers or with curl:

```bash
# Gazebo stats
curl -N http://localhost:8080/gz_controller/properties/simStats/observable

# Model poses
curl -N http://localhost:8080/gz_controller/properties/poses/observable

# CoppeliaSim stats
curl -N http://localhost:8081/cs_controller/properties/simStats/observable
```
note: if observing in web brosers, two helper buttons are defined in /test/bookmark_buttons.txt

### Container logs

```bash
# Container logs
docker logs WoT_Server
docker logs Gazebo_ROS2
docker logs CoppeliaSim_ROS2
```

### Testing

See [test/README.md](test/README.md)

### Development Reference

- **`native_api_and_ros2_calls.md`**: Comprehensive collection of native Gazebo and CoppeliaSim API calls, ROS2 commands, and service integrations used during development. Includes entity management, simulation control, robot manipulation, and debugging commands for both simulators.

## Troubleshooting

### Line Ending Issues on Windows Docker

If you encounter shell script execution errors on Windows Docker (e.g., `bad interpreter: No such file or directory`), this is typically due to CRLF vs LF line ending issues.

**Automatic fixes** (recommended):
- Docker build and entrypoint scripts automatically fix line endings
- No manual action needed in most cases

**Manual fixes** (if automatic fixes don't work):
- **Linux/Mac/WSL**: `./fix-line-endings.sh`
- **Windows**: `fix-line-endings.bat`

### General Issues

Check the logs either from docker logs or run the servers in development mode.

### UR10 Issues

- When loading ur10 models, you need to distinguish ur10_rg2_gazebo.urdf and ur10_rg2_coppelia.urdf for the corresponding simulators.

- If ur10 is not configured after loading in Gazebo (can't move the joints), you can start the containers in dev mode, run main/start_process_supervisor.sh in Gazebo_ROS2 container and gz_controller.js in WoT_Server container; then inside Gazebo_ROS2 container, manually run /Assets/urdf/examples/robots/ur10_rg2/ur10_config.sh, if the script is not correctly interpreted, see line ending issues troubleshooting above, after successful configuration you should see "you can start planning now"

## License

MIT License

## Authors

Yifan

## Related Projects

- [Web of Things (W3C)](https://www.w3.org/WoT/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/)
- [CoppeliaSim](https://www.coppeliarobotics.com/)
- [MoveIt 2](https://moveit.ros.org/)

