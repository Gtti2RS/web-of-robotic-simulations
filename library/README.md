# WoS Library Documentation

This directory contains the core library modules for the Web of Simulators (WoS) project, providing utilities and integrations for Gazebo simulation control via Web of Things (WoT) interfaces.

## Directory Structure

```
library/
├── common/           # Common utilities used across the project
│   ├── deg2quat.js           # Euler angle to quaternion conversion
│   ├── fileUtils.js          # File system operations and asset management
│   ├── ros2_service_helper.js # ROS 2 service and action call utilities
│   └── ros2_utils.js         # ROS 2 message publishing and command execution
└── gazebo/           # Gazebo-specific integrations
    ├── gz_bridge_manager.js  # ROS 2 bridge lifecycle management
    ├── gz_observable_topics.js # Real-time data streaming via SSE
    ├── gz_ros2_srv.js        # WoT action handlers for Gazebo control
    └── gz_world_utils.js     # World file and entity utilities
```

## Common Library (`common/`)

### `deg2quat.js`
**Purpose**: Convert Euler angles to quaternions for robotics and 3D simulations.

**Key Functions**:
- `deg2quat(orientation)` - Convert roll/pitch/yaw in degrees to quaternion

**Usage**:
```javascript
const { deg2quat } = require('./common/deg2quat');
const quat = deg2quat({ roll: 0, pitch: 90, yaw: 0 });
```

### `fileUtils.js`
**Purpose**: File system operations, asset discovery, and secure file upload handling.

**Key Functions**:
- `handleGazeboUpload()` - Secure file upload with validation
- `readGazeboAssets()` - Discover and organize URDF/Gazebo files
- `resolveFilePath()` - Resolve simulation file paths
- `checkFileConflict()` - Detect file conflicts

**Usage**:
```javascript
const { handleGazeboUpload, readGazeboAssets } = require('./common/fileUtils');
```

### `ros2_service_helper.js`
**Purpose**: Generic ROS 2 service and action caller with client reuse and payload validation.

**Key Functions**:
- `callService(node, { srvType, serviceName, payload }, opts)` - Call ROS 2 services
- `callAction(node, { actionType, actionName, goal }, opts)` - Call ROS 2 actions

**Features**:
- Client reuse per (srvType, serviceName)
- Payload key filtering
- Timeout handling
- Debug logging

### `ros2_utils.js`
**Purpose**: ROS 2 message publishing and secure command execution.

**Key Functions**:
- `makePublishMessage(node)` - Factory for ROS 2 message publishing
- `makeSendRos2Cmd(node, options)` - Factory for secure ROS 2 command execution

**Features**:
- Automatic publisher management
- Secure command execution with timeout
- SIGTERM → SIGKILL escalation
- Process tracking and cleanup

## Gazebo Library (`gazebo/`)

### `gz_ros2_srv.js`
**Purpose**: Main WoT action handlers for Gazebo simulation control via ROS 2 services.

**Key Action Handlers**:
- `makeLaunchSimulation()` - Launch Gazebo simulations
- `makeExitSimulation()` - Stop running simulations
- `makeSimControl()` - Control simulation (pause/run/reset)
- `makeSpawnEntity()` - Spawn entities in simulation
- `makeSetEntityPose()` - Set entity positions/orientations
- `makeRemoveEntity()` - Remove entities from simulation
- `makeSaveWorld()` - Save current world state
- `makeSetVisualization()` - Toggle camera visualization
- `makeSetRtf()` - Set real-time factor

**Features**:
- Factory pattern for node capture
- UR10 robot integration with automatic TD exposure
- Process supervision and cleanup
- Support for both SDF and launch files

### `gz_observable_topics.js`
**Purpose**: Real-time data streaming from Gazebo via ROS 2 topics using Server-Sent Events.

**Observable Properties**:
- `simStats` - Simulation statistics (timing, iterations, RTF)
- `poses` - Entity positions and orientations
- `models` - Available models in simulation

**Key Functions**:
- `readSimStats()` - Read simulation statistics
- `readPoses()` - Read entity poses
- `readModels()` - Read available models
- `combinedSSEMiddleware` - SSE middleware for web clients

**Features**:
- Real-time data streaming
- Automatic topic subscription management
- Connection lifecycle management
- Keep-alive heartbeat

### `gz_bridge_manager.js`
**Purpose**: Manage ROS 2 bridges for Gazebo communication.

**Bridge Types**:
- `world_services` - Entity management services
- `world_topics` - Simulation statistics and pose topics
- `image_bridge` - Camera stream visualization

**Key Functions**:
- `setupBridge()` - Start a specific bridge
- `stopBridge()` - Stop a specific bridge
- `setupBridges()` - Start multiple bridges for a world
- `stopAllBridges()` - Stop all active bridges

**Features**:
- Bridge registry system
- Automatic lifecycle management
- World-specific configuration
- Service-based process control

### `gz_world_utils.js`
**Purpose**: Utilities for working with Gazebo world files and entities.

**Key Functions**:
- `extract_world()` - Extract world name from files
- `get_world()` - Get current world name
- `clear_world()` - Clear stored world state
- `entityExists()` - Check if entity exists
- `ur10Exists()` - Check for UR10 robots
- `extractSdfFromLaunch()` - Extract SDF from launch files

**Features**:
- World name extraction from SDF/launch files
- Entity existence validation
- UR10 robot detection
- World state management

## Usage Examples

### Basic Simulation Control
```javascript
const { makeLaunchSimulation, makeExitSimulation } = require('./gazebo/gz_ros2_srv');

const launchSim = makeLaunchSimulation(node);
const exitSim = makeExitSimulation(node);

// Launch simulation
await launchSim({ value: async () => ({ fileName: 'my_world.sdf' }) });

// Exit simulation
await exitSim({ value: async () => ({}) });
```

### Real-time Data Streaming
```javascript
const { readSimStats, readPoses } = require('./gazebo/gz_observable_topics');

// Read simulation statistics
const stats = await readSimStats();
console.log('RTF:', stats.realTimeFactor);

// Read entity poses
const poses = await readPoses();
console.log('Entities:', poses.pose.length);
```

### Bridge Management
```javascript
const { setupBridges, stopAllBridges } = require('./gazebo/gz_bridge_manager');

// Setup bridges for a world
await setupBridges(node, 'my_world');

// Stop all bridges
await stopAllBridges(node);
```

## Architecture Notes

- **Factory Pattern**: Most functions return factories that capture ROS 2 node instances
- **Service-based**: Communication with Gazebo primarily via ROS 2 services
- **Process Management**: Automatic cleanup of child processes and services
- **Error Handling**: Comprehensive error handling with meaningful messages
- **Real-time**: SSE implementation for live data streaming to web clients

## Dependencies

- **rclnodejs**: ROS 2 JavaScript bindings
- **@node-wot/core**: Web of Things core library
- **fs/promises**: File system operations
- **path**: Path manipulation utilities

## Compatibility

- **ROS 2**: Jazzy
- **Gazebo**: Harmonic
- **Node.js**: 18+
- **Ubuntu**: 24.04.2 LTS
