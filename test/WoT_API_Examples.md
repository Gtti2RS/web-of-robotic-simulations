# WoT API Examples with curl

This document provides unified example curl requests for the Web of Things (WoT) APIs for both Gazebo and CoppeliaSim simulators.

## Variables

Replace the following variables in the examples:
- `{SERVER_URL}`: Server hostname or IP (e.g., `localhost`, `10.157.150.3`)
- `{PORT}`: Controller port (e.g., `8080` for Gazebo, `8081` for CoppeliaSim)
- `{CONTROLLER}`: Controller name (e.g., `gz_controller`, `cs_controller`)
- `{SCENE_FILE}`: Scene file name (e.g., `shapes.sdf` for Gazebo, `controlledViaRos2.ttt` for CoppeliaSim)
- `{MODEL_FILE}`: Model file name (e.g., `box.urdf`, `ur10_rg2_gazebo.urdf`, `ur10_rg2_coppelia.urdf`)
- `{MODEL_NAME}`: Model/object name (e.g., `robot1`, `ur10_rg2`)

## Base URL Format
```
http://{SERVER_URL}:{PORT}/{CONTROLLER}/
```

## Example URLs
- **Gazebo Controller**: `http://localhost:8080/gz_controller/`
- **CoppeliaSim Controller**: `http://localhost:8081/cs_controller/`
- **UR10 Server (Gazebo)**: `http://localhost:8083/ur10_server/`
- **UR10 Server (CoppeliaSim)**: `http://localhost:8084/ur10_server/`

---

## Simulator Controller API Examples

### 1. Scene Management

**Load a scene**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/manageScene" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "load",
    "fileName": "{SCENE_FILE}"
  }'
```

**Save current scene**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/manageScene" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "save",
    "fileName": "my_scene.{EXTENSION}"
  }'
```

**Close scene**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/manageScene" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "close"
  }'
```

### 2. Model Management

**Load a model**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/manageModel" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "load",
    "fileName": "{MODEL_FILE}",
    "modelName": "{MODEL_NAME}",
    "position": {
      "x": 0.5,
      "y": 0.0,
      "z": 0.0
    },
    "orientation": {
      "roll": 0,
      "pitch": 0,
      "yaw": 0
    }
  }'
```

**Remove a model**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/manageModel" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "remove",
    "modelName": "{MODEL_NAME}"
  }'
```

**Set model pose**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/manageModel" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "setPose",
    "id": "{MODEL_NAME}",
    "position": {
      "x": 1.0,
      "y": 0.5,
      "z": 0.2
    },
    "orientation": {
      "roll": 0,
      "pitch": 0,
      "yaw": 45
    }
  }'
```

### 3. Simulation Control

**Pause simulation**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "pause"
  }'
```

**Run simulation**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "run"
  }'
```

**Stop simulation**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "stop"
  }'
```

**Increase simulation speed**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "faster"
  }'
```

**Decrease simulation speed**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "slower"
  }'
```

### 4. ROS 2 Commands

**Send ROS 2 command**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/sendRos2Cmd" \
  -H "Content-Type: application/json" \
  -d '"ros2 topic list"'
```

**Publish ROS 2 message**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/{CONTROLLER}/actions/publishMessage" \
  -H "Content-Type: application/json" \
  -d '"Hello from WoT API"'
```

### 5. File Upload

**Upload file to Gazebo**
```bash
curl -X POST "http://{SERVER_URL}:8082/upload" \
  -F "file=@/path/to/your/file.sdf" \
  -F "simulator=gazebo"
```

**Upload file to CoppeliaSim**
```bash
curl -X POST "http://{SERVER_URL}:8082/upload" \
  -F "file=@/path/to/your/file.ttt" \
  -F "simulator=coppeliasim"
```

**Supported file types:**
- **Gazebo**: `.sdf` (worlds), `.urdf` (models)
- **CoppeliaSim**: `.ttt` (scenes), `.ttm` (models), `.urdf` (URDF models)

### 6. Properties

**Get available assets**
```bash
curl -X GET "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/assets"
```

**Get simulation statistics**
```bash
curl -X GET "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/simStats"
```

**Get current models**
```bash
curl -X GET "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/models"
```

**Get entity poses**
```bash
curl -X GET "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/poses"
```

**Get visualization status**
```bash
curl -X GET "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/visualize"
```

**Set visualization status (Gazebo only)**
```bash
curl -X PUT "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/visualize" \
  -H "Content-Type: application/json" \
  -d 'true'
```

### 7. Observable Properties (Server-Sent Events)

**Subscribe to simulation statistics**
```bash
curl -N "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/simStats/observable"
```

**Subscribe to model updates**
```bash
curl -N "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/models/observable"
```

**Subscribe to pose updates**
```bash
curl -N "http://{SERVER_URL}:{PORT}/{CONTROLLER}/properties/poses/observable"
```

---

## UR10 Robot Control API Examples

### 1. Robot Control

**Move to joint positions**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/ur10_server/actions/moveToJoint" \
  -H "Content-Type: application/json" \
  -d '{
    "shoulder_pan_joint": 0,
    "shoulder_lift_joint": -90,
    "elbow_joint": 0,
    "wrist_1_joint": -90,
    "wrist_2_joint": 0,
    "wrist_3_joint": 0
  }'
```

**Move to joint positions (partial - only specify some joints)**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/ur10_server/actions/moveToJoint" \
  -H "Content-Type: application/json" \
  -d '{
    "shoulder_pan_joint": 45,
    "shoulder_lift_joint": null,
    "elbow_joint": -45,
    "wrist_1_joint": null,
    "wrist_2_joint": null,
    "wrist_3_joint": null
  }'
```

**Move to Cartesian position (using roll/pitch/yaw)**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/ur10_server/actions/moveToCartesian" \
  -H "Content-Type: application/json" \
  -d '{
    "position": {
      "x": 0.5,
      "y": 0.0,
      "z": 0.5
    },
    "orientation": {
      "roll": 0,
      "pitch": 0,
      "yaw": 0
    }
  }'
```

**Move to Cartesian position (using quaternion)**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/ur10_server/actions/moveToCartesian" \
  -H "Content-Type: application/json" \
  -d '{
    "position": {
      "x": 0.3,
      "y": 0.2,
      "z": 0.4
    },
    "orientation": {
      "x": 0,
      "y": 0,
      "z": 0,
      "w": 1
    }
  }'
```

### 2. Gripper Control

**Open gripper**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/ur10_server/actions/gripOpen" \
  -H "Content-Type: application/json" \
  -d 'null'
```

**Close gripper**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/ur10_server/actions/gripClose" \
  -H "Content-Type: application/json" \
  -d 'null'
```

### 3. Emergency Control

**Emergency stop**
```bash
curl -X POST "http://{SERVER_URL}:{PORT}/ur10_server/actions/emergencyStop" \
  -H "Content-Type: application/json" \
  -d '{}'
```

### 4. Properties (Read-only)

**Get current joint positions**
```bash
curl -X GET "http://{SERVER_URL}:{PORT}/ur10_server/properties/jointPositions"
```

---

## Simulator-Specific Notes

### Gazebo
- Scene files use `.sdf` extension
- Model files use `.urdf` or `.sdf` extension
- ur10 model: `ur10_rg2_gazebo.urdf`

### CoppeliaSim
- Scene files use `.ttt` extension
- Model files use `.urdf` or `.ttm` extension
- ur10 model: `ur10_rg2_coppelia.urdf`
---

## Joint Limits for UR10 Robot

When using `moveToJoint`, keep in mind the following limits:

- **shoulder_pan_joint**: -180° to 180°
- **shoulder_lift_joint**: -180° to 0°
- **elbow_joint**: -140° to 140°
- **wrist_1_joint**: -180° to 180°
- **wrist_2_joint**: -180° to 180°
- **wrist_3_joint**: -180° to 180°

Use `null` for any joint you want to keep at its current position.

---

## Usage Examples

### Example 1: Load a robot in Gazebo
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/manageModel" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "load",
    "fileName": "ur10_rg2_gazebo.urdf",
    "modelName": "robot1",
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"roll": 0, "pitch": 0, "yaw": 0}
  }'
```

### Example 2: Load a robot in CoppeliaSim
```bash
curl -X POST "http://localhost:8081/cs_controller/actions/manageModel" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "load",
    "fileName": "ur10_rg2_coppelia.urdf",
    "modelName": "robot1",
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"roll": 0, "pitch": 0, "yaw": 0}
  }'
```

### Example 3: Control UR10 robot (Gazebo)
```bash
curl -X POST "http://localhost:8083/ur10_server/actions/moveToCartesian" \
  -H "Content-Type: application/json" \
  -d '{
    "position": {"x": -0.5, "y": 0.5, "z": 0.25},
    "orientation": {"roll": 0, "pitch": 90, "yaw": 0}
  }'
```

### Example 4: Control UR10 robot (CoppeliaSim)
```bash
curl -X POST "http://localhost:8084/ur10_server/actions/moveToCartesian" \
  -H "Content-Type: application/json" \
  -d '{
    "position": {"x": -0.5, "y": 0.5, "z": 0.25},
    "orientation": {"roll": 0, "pitch": 90, "yaw": 0}
  }'
```

### Example 5: Upload a world file to Gazebo
```bash
curl -X POST "http://localhost:8082/upload" \
  -F "file=@/path/to/test1626.sdf" \
  -F "simulator=gazebo"
```

### Example 6: Upload a scene file to CoppeliaSim
```bash
curl -X POST "http://localhost:8082/upload" \
  -F "file=@/path/to/controlledViaRos2.ttt" \
  -F "simulator=coppeliasim"
```

---

## Notes

1. All POST requests require `Content-Type: application/json` header
2. Observable properties use Server-Sent Events (SSE) - use `curl -N` to stream data
3. File uploads use multipart/form-data format
4. Replace variables `{SERVER_URL}`, `{PORT}`, `{CONTROLLER}`, etc. with actual values
5. Cartesian movements use inverse kinematics to compute joint positions, current config disables avoid_collision, so robot may move to normally unreachable positions

# Gazebo
- Stream only updates when simulation is running
- Use manageModel with mode setPose and modelName wot_camera (CAMERA_NAME in gz_ros2_srv.js) to set view angle, example poses:  
close to robot: "position": {"x": -1.5, "y": 0.0, "z": 0}, "orientation": {"roll": 0, pitch": 0, "yaw": 0}
broader: "position": {"x": -2, "y": 0.0, "z": 2}, "orientation": {"roll": 0, pitch": 40, "yaw": 0}
- The native service /world/.../control with request 'reset' can't reset the camera sensor cleanly, consequently, if visualization is enabled, calling simControl with mode "stop" will show dangling images from previous models and fixed view angle. A better solution would be save-close-load current scene with manageScene.
- User can use either sdf file or ros2 launch file for launching, in case using ros2 launch files, avoid trigger any process that requires gui if gazebo is running on headless server
- If user upload custom world files, plugins: Sensors, Physics, SceneBroadcaster, UserCommands should be included to enable all features, current files under Assets/gazebo/worlds/vendor originate from vendor folder "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds" and necessary plugins are added
- Current ur10 controller has weird planning when movement is larger than 120° for shoulder_pan_joint, as workaround a large movement (>90°) is splitted into smaller movements. After spawn ur10, the recommended first movement is calling moveToJoint with the following safe positions.In rare cases when shoulder_pan_joint spins crazily, call emergencyStop and then moveToJoint with the safe positions.
{
  "shoulder_pan_joint": null,
  "shoulder_lift_joint": -90,
  "elbow_joint": 90,
  "wrist_1_joint": -90,
  "wrist_2_joint": 0,
  "wrist_3_joint": 0
}

# CoppeliaSim
- Must close CoppeliaSim stream web page before starting coppeliasim, otherwise will get "address already in use" error
