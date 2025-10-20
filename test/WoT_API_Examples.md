# WoT API Examples with curl

This document provides example curl requests for the Web of Things (WoT) APIs defined in `gz_controller.json` and `ur10_server.json`.

## Base URLs
- **gz_controller**: `http://localhost:8080/gz_controller/`
- **ur10_server(gazebo)**: `http://localhost:8083/ur10_server/`
- **ur10_server(coppeliasim)**: `http://localhost:8084/ur10_server/`

---

## gz_controller API Examples

### 1. Scene Management

#### Load a scene
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/manageScene" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "load",
    "fileName": "empty.sdf"
  }'
```

#### Save current scene
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/manageScene" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "save",
    "fileName": "my_scene.sdf"
  }'
```

#### Close scene
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/manageScene" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "close"
  }'
```

### 2. Model Management

#### Load a model
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/manageModel" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "load",
    "fileName": "ur10_rg2_gazebo.urdf",
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

#### Remove a model
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/manageModel" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "remove",
    "modelName": "ur10_rg2"
  }'
```

#### Set model pose
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/manageModel" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "setPose",
    "id": "ur10_rg2",
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

#### Pause simulation
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "pause"
  }'
```

#### Run simulation
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "run"
  }'
```

#### Stop simulation
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "stop"
  }'
```

#### Increase simulation speed
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "faster"
  }'
```

#### Decrease simulation speed
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/simControl" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "slower"
  }'
```

### 4. ROS 2 Commands

#### Send ROS 2 command
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/sendRos2Cmd" \
  -H "Content-Type: application/json" \
  -d '"ros2 topic list"'
```

#### Publish ROS 2 message
```bash
curl -X POST "http://localhost:8080/gz_controller/actions/publishMessage" \
  -H "Content-Type: application/json" \
  -d '"Hello from WoT API"'
```

### 5. File Upload

#### Upload file
```bash
curl -X POST "http://10.157.150.3:8082/upload" \
  -F "file=@/path/to/your/file.urdf"
```

### 6. Properties (Read-only)

#### Get available assets
```bash
curl -X GET "http://localhost:8080/gz_controller/properties/assets"
```

#### Get simulation statistics
```bash
curl -X GET "http://localhost:8080/gz_controller/properties/simStats"
```

#### Get current models
```bash
curl -X GET "http://localhost:8080/gz_controller/properties/models"
```

#### Get entity poses
```bash
curl -X GET "http://localhost:8080/gz_controller/properties/poses"
```

#### Get visualization status
```bash
curl -X GET "http://localhost:8080/gz_controller/properties/visualize"
```

#### Set visualization status
```bash
curl -X PUT "http://localhost:8080/gz_controller/properties/visualize" \
  -H "Content-Type: application/json" \
  -d 'true'
```

### 7. Observable Properties (Server-Sent Events)

#### Subscribe to simulation statistics
```bash
curl -N "http://localhost:8080/gz_controller/properties/simStats/observable"
```

#### Subscribe to model updates
```bash
curl -N "http://localhost:8080/gz_controller/properties/models/observable"
```

#### Subscribe to pose updates
```bash
curl -N "http://localhost:8080/gz_controller/properties/poses/observable"
```

---

## ur10_server API Examples

### 1. Robot Control

#### Move to joint positions
```bash
curl -X POST "http://localhost:8080/ur10_server/actions/moveToJoint" \
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

#### Move to joint positions (partial - only specify some joints)
```bash
curl -X POST "http://localhost:8080/ur10_server/actions/moveToJoint" \
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

#### Move to Cartesian position (using roll/pitch/yaw)
```bash
curl -X POST "http://localhost:8080/ur10_server/actions/moveToCartesian" \
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

#### Move to Cartesian position (using quaternion)
```bash
curl -X POST "http://localhost:8080/ur10_server/actions/moveToCartesian" \
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

#### Open gripper
```bash
curl -X POST "http://localhost:8080/ur10_server/actions/gripOpen" \
  -H "Content-Type: application/json" \
  -d 'null'
```

#### Close gripper
```bash
curl -X POST "http://localhost:8080/ur10_server/actions/gripClose" \
  -H "Content-Type: application/json" \
  -d 'null'
```

### 3. Emergency Control

#### Emergency stop
```bash
curl -X POST "http://localhost:8080/ur10_server/actions/emergencyStop" \
  -H "Content-Type: application/json" \
  -d '{}'
```

### 4. Properties (Read-only)

#### Get current joint positions
```bash
curl -X GET "http://localhost:8080/ur10_server/properties/jointPositions"
```

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

## Notes

1. All POST requests require `Content-Type: application/json` header
2. Observable properties use Server-Sent Events (SSE) - use `curl -N` to stream data
3. File uploads use multipart/form-data format
4. The base URL `http://localhost:8080/` can be changed to match your server configuration
5. Some endpoints may require specific models or scenes to be loaded first
6. Emergency stop will immediately halt all robot movement
7. Cartesian movements use inverse kinematics to compute joint positions
