# UR10 RG2 Robot Model

This directory contains the UR10 robotic arm with RG2 gripper model files, configured for both Gazebo and CoppeliaSim simulators. In the urdf files, the ur10 part originates from the ros2 ur_description package and rg2 part originates from https://github.com/AndrejOrsula/ur5_rg2_ign

## 📁 File Overview

### URDF Files
- **`ur10_rg2_gazebo.urdf`** - URDF model optimized for Gazebo simulator
- **`ur10_rg2_coppelia.urdf`** - URDF model optimized for CoppeliaSim simulator

### Configuration Files
- **`ur10_config.sh`** - Shell script to configure robot state publisher for Gazebo
- **`ur10_controllers.yaml`** - ROS 2 controller configuration
- **`ur10_rg2_moveit_params.yaml`** - MoveIt motion planning configuration
- **`model.config`** - Gazebo model configuration file

### Server Applications
- **`ur10_server_gazebo.js`** - WoT server for Gazebo integration
- **`ur10_server_coppelia.js`** - WoT server for CoppeliaSim integration
- **`ur10_server.json`** - WoT Thing Description for the UR10 robot

### Mesh Assets
- **`meshes/`** - 3D mesh files for visual and collision geometry
  - `visual/` - High-quality visual meshes (.dae files)
  - `collision/` - Simplified collision meshes (.stl files)

## 🔄 Key Differences Between URDF Files

The CoppeliaSim version (`ur10_rg2_coppelia.urdf`) is a variant of the Gazebo version with specific modifications for CoppeliaSim's URDF importer:

1. **Removed `world` link**: CoppeliaSim's URDF importer generates a single model instead of discrete links when there's no world reference frame
2. **Modified `base_link`**: Added minimal inertial properties to `base_link` for CoppeliaSim compatibility
3. **Removed `base_joint`**: No world-to-base connection needed since there's no world link


## ⚠️ Important Notes

1. **File Suffix Validation**: The system enforces correct URDF usage:
   - Gazebo simulator only accepts `ur10_rg2_gazebo.urdf`
   - CoppeliaSim simulator only accepts `ur10_rg2_coppelia.urdf`
   - Attempting to use the wrong file will result in validation errors

2. **Mesh Paths**: All mesh files use absolute paths:
   ```
   file:///project-root/Assets/urdf/examples/robots/ur10_rg2/meshes/...
   ```

3. **ROS 2 Control**: Both URDF files are configured for ROS 2 control integration with identical controller configurations.

## 🔧 Development

When modifying the robot model:
1. Update both URDF files to maintain consistency
2. Test in both Gazebo and CoppeliaSim environments
3. Ensure mesh paths remain valid
4. Update MoveIt parameters if kinematic changes are made

## 📚 References

- [UR10 Robot Documentation](https://www.universal-robots.com/products/ur10-robot/)
- [RG2 Gripper Documentation](https://onrobot.com/en/products/rg2-finger-gripper)
- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [ROS 2 Control](https://control.ros.org/)
