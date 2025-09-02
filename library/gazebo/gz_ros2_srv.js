// Export WoT action handlers as factories that capture your already-created rclnodejs node.

const { callService } = require('../common/ros2_service_helper');
const { resolveFilePath } = require('../common/fileUtils');
const { deg2quat } = require('../common/deg2quat');
const { get_world, extract_world, clear_world } = require('./gz_world_utils');
const { entityExists } = require('./gz_world_utils'); 
const path = require('path'); // Added for makeSaveWorld
const fs = require('fs'); // Added for makeSaveWorld

const CAMERA_NAME = 'wot_camera';
let vizState = false;
let simProcessName = null; // Track the simulation process name

/**
 * Set real-time factor (RTF) for the active world.
 * Uses minimal payload (real_time_factor) and optionally max_step_size if provided.
 */
function makeSetRtf(node, { timeoutMs = 1000 } = {}) {
  return async function setRtf(input) {
    const { rtf, maxStep } = await input.value();
    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    const { req, resp } = await callService(
      node,
      {
        srvType: 'gz_physics_bridge/srv/SetPhysics',
        serviceName: `/world/${world}/set_physics`,
        payload: {
          real_time_factor: Number(rtf),
          max_step_size: Number.isFinite(maxStep) ? Number(maxStep) : undefined
        }
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      const parts = [`RTF=${req.real_time_factor}`, `world=${world}`];
      if ('max_step_size' in req && Number.isFinite(req.max_step_size)) {
        parts.splice(1, 0, `max_step_size=${req.max_step_size}`);
      }
      return parts.join(', ');
    }
    throw new Error(resp?.message ? `SetRtf failed: ${resp.message}` : 'SetRtf failed');
  };
}



/**
 * Remove an entity from the active world.
 * Uses minimal payload (name) and optionally typce if provided.
 */
function makeDeleteEntity(node, { timeoutMs = 1000 } = {}) {
  return async function deleteEntityAction(input) {
    const { id, name, type } = (await input.value()) ?? {};
    if (id == null && !name) {
      throw new Error('DeleteEntity requires either "id" or "name".');
    }

    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    // Verify the entity exists before attempting deletion
    if (id != null) {
      const exists = await entityExists(id);
      if (!exists) throw new Error(`Entity id ${id} not found.`);
    } else if (name) {
      const exists = await entityExists(name);
      if (!exists) throw new Error(`Entity ${name} not found.`);
    }

    const entity = {};
    if (id != null) {
      entity.id = (typeof id === 'bigint') ? id : BigInt(id);
    }
    if (name) entity.name = name;
    if (typeof type === 'number') entity.type = type;
    else if (name && id == null) entity.type = 2; // Default MODEL=2 when deleting by name

    const { req, resp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/DeleteEntity',
        serviceName: `/world/${world}/remove`,
        payload: { entity }
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      const identifier = name ? `name=${name}` : `id=${id}`;
      const typeInfo = type !== undefined ? `, type=${type}` : '';
      return `Deleted entity (${identifier}${typeInfo}).`;
    }
    throw new Error(resp?.message ? `DeleteEntity failed: ${resp.message}` : 'DeleteEntity failed');
  };
}



/**
 * Set entity pose via ROS 2 bridged service (ros_gz_interfaces/SetEntityPose)
 */
function makeSetEntityPose(node, { timeoutMs = 1000 } = {}) {
  return async function setEntityPoseAction(input) {
    const data = await input.value();

    const { id, name } = data ?? {};
    if (id == null && !name) throw new Error('SetEntityPose requires either "id" or "name".');

    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    // Existence check (supports id or name)
    const exists = await entityExists(id != null ? id : name);
    if (!exists) {
      throw new Error(id != null ? `Entity id ${id} not found.` : `Entity ${name} not found.`);
    }

    // Position defaults
    const p = data.position || {};
    const position = {
      x: Number.isFinite(p.x) ? p.x : 0,
      y: Number.isFinite(p.y) ? p.y : 0,
      z: Number.isFinite(p.z) ? p.z : 0
    };

    // Orientation: roll, pitch, yaw (deg) -> quaternion
    const rpy = data.orientation || {};
    const { qx, qy, qz, qw } = deg2quat({
      roll: Number.isFinite(rpy.roll) ? rpy.roll : 0,
      pitch: Number.isFinite(rpy.pitch) ? rpy.pitch : 0,
      yaw: Number.isFinite(rpy.yaw) ? rpy.yaw : 0
    });

    // Build request according to ros_gz_interfaces/srv/SetEntityPose
    const entity = {};
    if (id != null) entity.id = (typeof id === 'bigint') ? id : BigInt(id);
    if (name) entity.name = name;
    if (id == null && name) entity.type = 2; // MODEL when targeting by name

    const payload = {
      entity,
      pose: {
        position,
        orientation: { x: qx, y: qy, z: qz, w: qw }
      }
    };

    const { resp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/SetEntityPose',
        serviceName: `/world/${world}/set_pose`,
        payload
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      const identifier = name ? `name=${name}` : `id=${id}`;
      return `Pose set for entity (${identifier}).`;
    }
    throw new Error(resp?.message ? `SetEntityPose failed: ${resp.message}` : 'SetEntityPose failed');
  };
}

/**
 * Spawn entity via ROS 2 bridged service (ros_gz_interfaces/EntityFactory on /world/<world>/create)
 * Input mirrors existing spawn_entity action: { entity_name, file_name, position, orientation }
 */
function makeSpawnEntity(node, { timeoutMs = 1000 } = {}) {
  return async function spawnEntityAction(input) {
    const data = await input.value();

    const entity_name = data?.entity_name;
    const file_name = data?.file_name;
    if (!entity_name) throw new Error('SpawnEntity requires "entity_name".');
    if (!file_name) throw new Error('SpawnEntity requires "file_name".');

    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    // Avoid name collision
    const exists = await entityExists(entity_name);
    if (exists) return `Entity ${entity_name} already exists.`;

    const fullPath = await resolveFilePath(file_name);
    if (!fullPath) throw new Error(`File not found: ${file_name}`);

    // Position defaults
    const p = data.position || {};
    const position = {
      x: Number.isFinite(p.x) ? p.x : 0,
      y: Number.isFinite(p.y) ? p.y : 0,
      z: Number.isFinite(p.z) ? p.z : 0
    };

    // Orientation: roll/pitch/yaw (deg) -> quaternion
    const rpy = data.orientation || {};
    const { qx, qy, qz, qw } = deg2quat({
      roll: Number.isFinite(rpy.roll) ? rpy.roll : 0,
      pitch: Number.isFinite(rpy.pitch) ? rpy.pitch : 0,
      yaw: Number.isFinite(rpy.yaw) ? rpy.yaw : 0
    });

    // ros_gz_interfaces/srv/SpawnEntity expects an EntityFactory inside `entity_factory`
    const payload = {
      entity_factory: {
        name: entity_name,
        allow_renaming: false,
        sdf: undefined,
        sdf_filename: fullPath,
        clone_name: undefined,
        pose: {
          position,
          orientation: { x: qx, y: qy, z: qz, w: qw }
        },
        relative_to: 'world'
      }
    };

    const { resp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/SpawnEntity',
        serviceName: `/world/${world}/create`,
        payload
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      return `${entity_name} is spawned to current world.`;
    }
    throw new Error(resp?.message ? `SpawnEntity failed: ${resp.message}` : 'SpawnEntity failed');
  };
}

/**
 * Control simulation via ROS 2 bridged service (ros_gz_interfaces/ControlWorld)
 * Input: { mode: 'pause' | 'run' | 'resume' | 'reset' }
 */
function makeSimControl(node, { timeoutMs = 1000 } = {}) {
  return async function simControlAction(input) {
    const { mode } = await input.value();

    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    let world_control;
    switch (mode) {
      case 'pause':
        world_control = { pause: true };
        break;
      case 'run':
      case 'resume':
        world_control = { pause: false };
        break;
      case 'reset':
        world_control = { reset: { all: true }, pause: true };
        break;
      default:
        throw new Error(`Invalid mode: ${mode}`);
    }

    const { resp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/ControlWorld',
        serviceName: `/world/${world}/control`,
        payload: { world_control }
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      return `Simulation "${world}" successfully ${mode}d.`;
    }
    throw new Error(resp?.message ? `SimControl failed: ${resp.message}` : 'SimControl failed');
  };
}

/**
 * Read handler for WoT Property `visualization`.
 * Keeps state inside this module.
 */
function visualizationRead() {
  return vizState;
}

/**
 * Toggle visualization by spawning/removing camera model via ROS 2 services.
 * Input: boolean (true to enable, false to disable)
 */
function makeSetVisualization(node, { timeoutMs = 1000 } = {}) {
  return async function setVisualizationAction(input) {
    const desired = await input.value();
    
    if (desired == vizState) {
      return `Visualization already ${vizState ? 'enabled' : 'disabled'}`;
    }

    if (desired) {
      // Spawn camera if not exists
      if (!(await entityExists(CAMERA_NAME))) {
        const spawn_camera = makeSpawnEntity(node, { timeoutMs });
        await spawn_camera({ value: async () => ({
          entity_name: CAMERA_NAME,
          file_name: 'camera.sdf',
          position: { x: -10, y: 0, z: 10 },
          orientation: { roll: 0, pitch: 40, yaw: 0 }
        }) });
      }
      
      // Start bridge using ROS2 process management service
      try {
        const world = get_world();
        const gzTopic = `/world/${world}/model/${CAMERA_NAME}/link/link/sensor/camera/image`;
        const bridgeCmd = `ros2 run ros_gz_bridge parameter_bridge ${gzTopic}@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r ${gzTopic}:=/viz_cam`;
        
        const { resp: bridgeResp } = await callService(
          node,
          {
            srvType: 'sim_process_supervisor_interfaces/srv/ManagedStart',
            serviceName: '/process/managed/start',
            payload: {
              name: 'camera_bridge',
              cmd: bridgeCmd
            }
          },
          { timeoutMs }
        );
        
        if (bridgeResp?.success ?? bridgeResp?.ok ?? bridgeResp?.boolean) {
          console.log(`[makeSetVisualization] Bridge process started via ROS2 service`);
        } else {
          console.warn(`[makeSetVisualization] Failed to start bridge process: ${bridgeResp?.message || 'Unknown error'}`);
        }
      } catch (error) {
        console.error(`[makeSetVisualization] Error starting bridge process: ${error.message}`);
      }
      
      // Start web_video_server using ROS2 process management service
      try {
        const webVideoCmd = `ros2 run web_video_server web_video_server --ros-args -p port:=8081 -p address:=0.0.0.0 -p server_threads:=2 -p ros_threads:=3 -p default_stream_type:=mjpeg`;
        
        const { resp: webVideoResp } = await callService(
          node,
          {
            srvType: 'sim_process_supervisor_interfaces/srv/ManagedStart',
            serviceName: '/process/managed/start',
            payload: {
              name: 'web_video_server',
              cmd: webVideoCmd
            }
          },
          { timeoutMs }
        );
        
        if (webVideoResp?.success ?? webVideoResp?.ok ?? webVideoResp?.boolean) {
          console.log(`[makeSetVisualization] Web video server process started via ROS2 service`);
        } else {
          console.warn(`[makeSetVisualization] Failed to start web video server process: ${webVideoResp?.message || 'Unknown error'}`);
        }
      } catch (error) {
        console.error(`[makeSetVisualization] Error starting web video server process: ${error.message}`);
      }
      
      vizState = true;
      console.log(`[makeSetVisualization] Visualization enabled`);
      return 'Visualization enabled (camera and streams ensured via ROS2 process management).';
    } else {
      // Remove camera if exists
      if (await entityExists(CAMERA_NAME)) {
        const remove_camera = makeDeleteEntity(node, { timeoutMs });
        await remove_camera({ value: async () => ({ name: CAMERA_NAME }) });
      }
      
      // Stop bridge process using ROS2 process management service
      try {
        const { resp: bridgeResp } = await callService(
          node,
          {
            srvType: 'sim_process_supervisor_interfaces/srv/ManagedStop',
            serviceName: '/process/managed/stop',
            payload: {
              name: 'camera_bridge'
            }
          },
          { timeoutMs }
        );
        
        if (bridgeResp?.success ?? bridgeResp?.ok ?? bridgeResp?.boolean) {
          console.log(`[makeSetVisualization] Bridge process stopped via ROS2 service`);
        } else {
          console.warn(`[makeSetVisualization] Failed to stop bridge process: ${bridgeResp?.message || 'Unknown error'}`);
        }
      } catch (error) {
        console.error(`[makeSetVisualization] Error stopping bridge process: ${error.message}`);
      }
      
      // Stop web video server process using ROS2 process management service
      try {
        const { resp: webVideoResp } = await callService(
          node,
          {
            srvType: 'sim_process_supervisor_interfaces/srv/ManagedStop',
            serviceName: '/process/managed/stop',
            payload: {
              name: 'web_video_server'
            }
          },
          { timeoutMs }
        );
        
        if (webVideoResp?.success ?? webVideoResp?.ok ?? webVideoResp?.boolean) {
          console.log(`[makeSetVisualization] Web video server process stopped via ROS2 service`);
        } else {
          console.warn(`[makeSetVisualization] Failed to stop web video server process: ${webVideoResp?.message || 'Unknown error'}`);
        }
      } catch (error) {
        console.error(`[makeSetVisualization] Error stopping web video server process: ${error.message}`);
      }
      
      vizState = false;
      console.log(`[makeSetVisualization] Visualization disabled`);
      return 'Visualization disabled (camera removed if present, processes stopped via ROS2 service).';
    }
  };
}

/**
 * Save world SDF via ROS 2 bridged service (gz_physics_bridge/srv/GenerateWorldSdf)
 * Input: { name: string } - optional filename for the saved world
 */
function makeSaveWorld(node, { timeoutMs = 1000 } = {}) {
  return async function saveWorldAction(input) {
    const { name } = await input.value();
    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    const { resp } = await callService(
      node,
      {
        srvType: 'gz_physics_bridge/srv/GenerateWorldSdf',
        serviceName: `/world/${world}/generate_world_sdf`,
        payload: {} // Empty payload as per the service definition
      },
      { timeoutMs }
    );

    if (resp?.sdf) {
      const sdfString = resp.sdf;
      
      // Generate safe filename
      const safeName = name && name.trim() !== ''
        ? name.trim().replace(/[^a-zA-Z0-9_\-]/g, '_') + '.sdf'
        : `world_${world}_${new Date().toISOString().replace(/[:.]/g, '-')}.sdf`;

      const filePath = path.join(__dirname, '../../saved/world', safeName);
      const finalSdf = `<?xml version="1.0" ?>\n${sdfString}`;
      
      // Use fs to save the file
      const dir = path.dirname(filePath);
      if (!fs.existsSync(dir)) {
        fs.mkdirSync(dir, { recursive: true });
      }
      fs.writeFileSync(filePath, finalSdf);

      console.log(`World '${world}' saved to '${safeName}'`);
      return `World saved to '${filePath}'`;
    }
    throw new Error('Failed to generate world SDF');
  };
}

/**
 * Launch simulation using ROS2 process management service
 * Input: { method: string, fileName: string, arguments: array }
 */
function makeLaunchSimulation(node, { timeoutMs = 1000 } = {}) {
  return async function launchSimulationAction(input) {
    const data = await input.value();
    const method = data?.method || "gz";
    const fileName = data?.fileName;
    const args = data?.arguments || [];

    if (!fileName) throw new Error("Missing fileName for launchSimulation");

    const fullPath = await resolveFilePath(fileName);
    if (!fullPath) throw new Error(`File "${fileName}" not found in resource or upload folders`);

    if (simProcessName) throw new Error("Simulation already running");

    const cmd = method === "gz" 
      ? "gz sim" 
      : "ros2 launch";
    const cmdArgs = method === "gz"
      ? [fullPath, ...args]
      : [fullPath, ...args];

    const fullCmd = `${cmd} ${cmdArgs.join(" ")}`;
    const processName = `simulation_${Date.now()}`; // Unique process name

    console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] Starting simulation: ${fullCmd}`);

    try {
      const { resp } = await callService(
        node,
        {
          srvType: 'sim_process_supervisor_interfaces/srv/ManagedStart',
          serviceName: '/process/managed/start',
          payload: {
            name: processName,
            cmd: fullCmd
          }
        },
        { timeoutMs }
      );

      if (resp?.success ?? resp?.ok ?? resp?.boolean) {
        simProcessName = processName;
        
        // Wait a bit for simulation to start
        await new Promise(resolve => setTimeout(resolve, 2000));

        try {
          // Extract world name from the SDF file
          const worldName = await extract_world(fullPath);
          console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] World name extracted: ${worldName}`);
          
          return `Simulation launched: ${fullPath}, world: ${worldName}, process: ${processName}`;
        } catch (detectErr) {
          console.warn(`[${new Date().toISOString()}] [makeLaunchSimulation] World detection failed:`, detectErr.message);
          return `Simulation launched: ${fullPath}, process: ${processName}, but world detection failed`;
        }
      } else {
        throw new Error(`Failed to start simulation: ${resp?.message || 'Unknown error'}`);
      }
    } catch (error) {
      console.error(`[${new Date().toISOString()}] [makeLaunchSimulation] Error:`, error);
      throw new Error('Simulation launch failed: ' + error.message);
    }
  };
}

/**
 * Exit simulation using ROS2 process management service
 * Input: {} - no input required
 */
function makeExitSimulation(node, { timeoutMs = 1000 } = {}) {
  return async function exitSimulationAction(input) {
    console.log(`[${new Date().toISOString()}] [makeExitSimulation] Starting simulation exit`);
    
    if (!simProcessName) {
      console.log(`[${new Date().toISOString()}] [makeExitSimulation] No simulation is currently running`);
      return "No simulation is currently running.";
    }

    try {
      const { resp } = await callService(
        node,
        {
          srvType: 'sim_process_supervisor_interfaces/srv/ManagedStop',
          serviceName: '/process/managed/stop',
          payload: {
            name: simProcessName
          }
        },
        { timeoutMs }
      );

      if (resp?.success ?? resp?.ok ?? resp?.boolean) {
        console.log(`[${new Date().toISOString()}] [makeExitSimulation] Simulation process stopped via ROS2 service`);
      } else {
        console.warn(`[${new Date().toISOString()}] [makeExitSimulation] Failed to stop simulation process: ${resp?.message || 'Unknown error'}`);
      }
    } catch (error) {
      console.error(`[${new Date().toISOString()}] [makeExitSimulation] Error stopping simulation process: ${error.message}`);
    }

    // Clean up state
    simProcessName = null;
    clear_world(); // Clear the module's stored world name
    vizState = false;

    // Stop any visualization processes if they're running
    try {
      // Stop bridge process
      const { resp: bridgeResp } = await callService(
        node,
        {
          srvType: 'sim_process_supervisor_interfaces/srv/ManagedStop',
          serviceName: '/process/managed/stop',
          payload: {
            name: 'camera_bridge'
          }
        },
        { timeoutMs }
      );
      
      if (bridgeResp?.success ?? bridgeResp?.ok ?? bridgeResp?.boolean) {
        console.log(`[makeExitSimulation] Bridge process stopped via ROS2 service`);
      }
    } catch (error) {
      console.error(`[makeExitSimulation] Error stopping bridge process: ${error.message}`);
    }

    try {
      // Stop web video server process
      const { resp: webVideoResp } = await callService(
        node,
        {
          srvType: 'sim_process_supervisor_interfaces/srv/ManagedStop',
          serviceName: '/process/managed/stop',
          payload: {
            name: 'web_video_server'
          }
        },
        { timeoutMs }
      );
      
      if (webVideoResp?.success ?? webVideoResp?.ok ?? webVideoResp?.boolean) {
        console.log(`[makeExitSimulation] Web video server process stopped via ROS2 service`);
      }
    } catch (error) {
      console.error(`[makeExitSimulation] Error stopping web video server process: ${error.message}`);
    }

    console.log(`[${new Date().toISOString()}] [makeExitSimulation] Simulation exit completed`);
    return "Simulation exited.";
  };
}

module.exports = {
  makeSetRtf,
  makeDeleteEntity,
  makeSetEntityPose,
  makeSpawnEntity,
  makeSimControl,
  makeSetVisualization,
  makeSaveWorld,
  makeLaunchSimulation,
  makeExitSimulation,
  visualizationRead,
};
