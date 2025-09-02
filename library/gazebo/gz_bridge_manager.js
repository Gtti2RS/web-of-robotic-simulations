const { callService } = require('../common/ros2_service_helper');

// Bridge registry - defines available bridges
const BRIDGE_REGISTRY = {
  world_services: {
    name: 'world_services_bridge',
    cmd: (worldName) => `ros2 run ros_gz_bridge parameter_bridge \\
      /world/${worldName}/create@ros_gz_interfaces/srv/SpawnEntity \\
      /world/${worldName}/remove@ros_gz_interfaces/srv/DeleteEntity \\
      /world/${worldName}/set_pose@ros_gz_interfaces/srv/SetEntityPose \\
      /world/${worldName}/control@ros_gz_interfaces/srv/ControlWorld`,
    description: 'World services bridge for entity management'
  },
  physics_bridge: {
    name: 'physics_bridge',
    cmd: (worldName) => `ros2 launch gz_physics_bridge bridge_throttled.launch.py world_name:=${worldName} stats_rate:=1 pose_rate:=1`,
    description: 'Physics bridge for throttled topics'
  },
  image_bridge: {
    name: 'camera_bridge',
    cmd: (worldName) => `ros2 run ros_gz_bridge parameter_bridge /world/${worldName}/model/wot_camera/link/link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r /world/${worldName}/model/wot_camera/link/link/sensor/camera/image:=/viz_cam`,
    description: 'Image bridge for camera visualization'
  }
};

// No need to track processes - package handles that

/**
 * Setup a specific bridge
 * @param {Object} node - ROS2 node
 * @param {string} bridgeType - Type of bridge from BRIDGE_REGISTRY
 * @param {string} worldName - World name for the bridge
 * @param {Object} options - Options including timeoutMs
 * @returns {Promise<string|null>} Process name if successful, null if failed
 */
async function setupBridge(node, bridgeType, worldName, { timeoutMs = 1000 } = {}) {
  const bridgeConfig = BRIDGE_REGISTRY[bridgeType];
  if (!bridgeConfig) {
    console.warn(`[${new Date().toISOString()}] [setupBridge] Unknown bridge type: ${bridgeType}`);
    return null;
  }

  if (!worldName) {
    console.warn(`[${new Date().toISOString()}] [setupBridge] No world name provided for ${bridgeType} bridge`);
    return null;
  }

  const processName = bridgeConfig.name;
  const cmd = bridgeConfig.cmd(worldName);

  console.log(`[${new Date().toISOString()}] [setupBridge] Starting ${bridgeType} bridge: ${processName}`);
  console.log(`[${new Date().toISOString()}] [setupBridge] Command: ${cmd}`);

  try {
    const { resp } = await callService(
      node,
      {
        srvType: 'sim_process_supervisor_interfaces/srv/ManagedStart',
        serviceName: '/process/managed/start',
        payload: {
          name: processName,
          cmd: cmd
        }
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      console.log(`[${new Date().toISOString()}] [setupBridge] ${bridgeType} bridge started successfully: ${processName}`);
      return processName;
    } else {
      console.warn(`[${new Date().toISOString()}] [setupBridge] Failed to start ${bridgeType} bridge for world '${worldName}': ${resp?.message || 'Unknown error'}`);
      return null;
    }
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [setupBridge] Error starting ${bridgeType} bridge:`, error.message);
    return null;
  }
}

/**
 * Setup all bridges for a world
 * @param {Object} node - ROS2 node
 * @param {string} worldName - World name
 * @param {Array<string>} bridgeTypes - Array of bridge types to setup (defaults to all)
 * @param {Object} options - Options including timeoutMs
 * @returns {Promise<Array<string>>} Array of successful process names
 */
async function setupBridges(node, worldName, bridgeTypes = Object.keys(BRIDGE_REGISTRY), { timeoutMs = 1000 } = {}) {
  console.log(`[${new Date().toISOString()}] [setupBridges] Setting up bridges for world: ${worldName}`);
  console.log(`[${new Date().toISOString()}] [setupBridges] Bridge types: ${bridgeTypes.join(', ')}`);

  const successfulProcesses = [];

  for (const bridgeType of bridgeTypes) {
    const processName = await setupBridge(node, bridgeType, worldName, { timeoutMs });
    if (processName) {
      successfulProcesses.push(processName);
    }
  }

  console.log(`[${new Date().toISOString()}] [setupBridges] Successfully started ${successfulProcesses.length}/${bridgeTypes.length} bridges`);
  return successfulProcesses;
}

/**
 * Stop a specific bridge by its type
 * @param {Object} node - ROS2 node
 * @param {string} bridgeType - Type of bridge to stop
 * @param {Object} options - Options including timeoutMs
 * @returns {Promise<boolean>} True if successful
 */
async function stopBridge(node, bridgeType, { timeoutMs = 1000 } = {}) {
  const bridgeConfig = BRIDGE_REGISTRY[bridgeType];
  if (!bridgeConfig) {
    console.warn(`[${new Date().toISOString()}] [stopBridge] Unknown bridge type: ${bridgeType}`);
    return false;
  }

  const processName = bridgeConfig.name; // Use fixed process name
  console.log(`[${new Date().toISOString()}] [stopBridge] Stopping ${bridgeType} bridge: ${processName}`);

  try {
    
    const { resp } = await callService(
      node,
      {
        srvType: 'sim_process_supervisor_interfaces/srv/ManagedStop',
        serviceName: '/process/managed/stop',
        payload: {
          name: processName
        }
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      console.log(`[${new Date().toISOString()}] [stopBridge] ${bridgeType} bridge stopped successfully: ${processName}`);
      return true;
    } else {
      console.warn(`[${new Date().toISOString()}] [stopBridge] Failed to stop ${bridgeType} bridge '${processName}': ${resp?.message || 'Unknown error'}`);
      return false;
    }
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [stopBridge] Error stopping ${bridgeType} bridge:`, error.message);
    return false;
  }
}

/**
 * Stop all bridge processes by their command patterns
 * @param {Object} node - ROS2 node
 * @param {Object} options - Options including timeoutMs
 * @returns {Promise<number>} Number of bridges successfully stopped
 */
async function stopAllBridges(node, { timeoutMs = 1000 } = {}) {
  console.log(`[${new Date().toISOString()}] [stopAllBridges] Stopping all bridge processes...`);

  let stoppedCount = 0;

  // Stop all bridge processes by their fixed names
  for (const [bridgeType, config] of Object.entries(BRIDGE_REGISTRY)) {
    try {
      const processName = config.name; // Use fixed process name
      
      console.log(`[${new Date().toISOString()}] [stopAllBridges] Stopping ${bridgeType} bridge: ${processName}`);
      
      const { resp } = await callService(
        node,
        {
          srvType: 'sim_process_supervisor_interfaces/srv/ManagedStop',
          serviceName: '/process/managed/stop',
          payload: {
            name: processName
          }
        },
        { timeoutMs }
      );

      if (resp?.success ?? resp?.ok ?? resp?.boolean) {
        console.log(`[${new Date().toISOString()}] [stopAllBridges] ${bridgeType} bridge stopped successfully`);
        stoppedCount++;
      } else {
        console.warn(`[${new Date().toISOString()}] [stopAllBridges] Failed to stop ${bridgeType} bridge: ${resp?.message || 'Unknown error'}`);
      }
    } catch (error) {
      console.warn(`[${new Date().toISOString()}] [stopAllBridges] Error stopping ${bridgeType} bridge:`, error.message);
    }
  }

  console.log(`[${new Date().toISOString()}] [stopAllBridges] Stopped ${stoppedCount} bridge processes`);
  return stoppedCount;
}



module.exports = {
  setupBridge,
  setupBridges,
  stopBridge,
  stopAllBridges,
  getAvailableBridgeTypes: () => Object.keys(BRIDGE_REGISTRY),
  getBridgeConfig: (bridgeType) => BRIDGE_REGISTRY[bridgeType] || null,
  BRIDGE_REGISTRY
};
