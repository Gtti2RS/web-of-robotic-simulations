// Export WoT action handlers as factories that capture your already-created rclnodejs node.

const { callService } = require('../common/ros2_service_helper');
const { resolveFilePath, checkFileConflict } = require('../common/fileUtils');
const { deg2quat } = require('../common/deg2quat');
const { get_world, extract_world, clear_world, extractSdfFromLaunch, ur10Exists } = require('./gz_world_utils');
const { entityExists } = require('./gz_world_utils'); 
const { setupAllObservableProperties, cleanupSubscriptions } = require('./gz_topics');
const { setupBridge, stopBridge, setupBridges, stopAllBridges } = require('./gz_bridge_manager');
const path = require('path'); // Added for makeSaveWorld
const fs = require('fs'); // Added for makeSaveWorld
const { spawn } = require('child_process'); // Added for UR10 controller child process

const CAMERA_NAME = 'wot_camera';
let vizState = false;
let simProcessName = null; // Track the simulation process name
let topicSubscriptions = []; // Track topic subscriptions
let ur10Processes = new Map(); // Track UR10 processes by entity name: { config: processName, controller: childProcess }

// Helper function to stop web video server
async function stopWebVideoServer(node, { timeoutMs = 1000 } = {}) {
  try {
    const { resp } = await callService(
      node,
      {
        srvType: 'sim_process_supervisor_interfaces/srv/ManagedStop',
        serviceName: '/process/managed/stop',
        payload: { name: 'web_video_server' }
      },
      { timeoutMs }
    );
  } catch (error) {
    // Ignore errors when stopping web video server
  }
}


/**
 * Set real-time factor (RTF) for the active world.
 * Uses minimal payload (real_time_factor) and optionally max_step_size if provided.
 */
function makeSetRtf(node, { timeoutMs = 1000 } = {}) {
  return async function setRtf(input) {
    const { rtf, maxStep } = await input.value();
    const world = get_world();
    if (!world) throw new Error('No active world found');

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

    const world = get_world();
    if (!world) throw new Error('No active world found');

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
      let resultMessage = `Deleted entity (${identifier}${typeInfo}).`;
      
      // Handle UR10 processes if entity has a name
      if (name) {
        const ur10Message = await handleUr10Deletion(node, name, { timeoutMs });
        if (ur10Message) {
          resultMessage += ur10Message;
        }
      }
      
      return resultMessage;
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

    const world = get_world();
    if (!world) throw new Error('No active world found');

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

    const world = get_world();
    if (!world) throw new Error('No active world found');

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
      let resultMessage = `${entity_name} is spawned to current world.`;
      
      // Handle UR10 processes if file_name matches 'ur10_rg2.urdf'
      const ur10Message = await handleUr10Spawning(node, entity_name, file_name, { timeoutMs });
      if (ur10Message) {
        resultMessage += ur10Message;
      }
      
      return resultMessage;
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

    const world = get_world();
    if (!world) throw new Error('No active world found');

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
        // fix gazebo reset bug to enable clean reset of visualization
        await makeResetVisualization(node, { timeoutMs });
        
        // The control service is now called inside makeResetVisualization
        // Return success message directly
        return `Simulation "${world}" successfully reset.`;
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
 * Reset visualization by calling Gazebo control service and refreshing visualization if enabled.
 * Input: {} - no input required
 */
async function makeResetVisualization(node, { timeoutMs = 1000 } = {}) {
  console.log(`[${new Date().toISOString()}] [makeResetVisualization] Starting visualization reset`);
  
  const world = get_world();
  if (!world) {
    console.warn(`[${new Date().toISOString()}] [makeResetVisualization] No active world found, skipping visualization reset`);
    return;
  }

  try {
    // Call Gazebo control service to reset the world
    console.log(`[${new Date().toISOString()}] [makeResetVisualization] Calling Gazebo control service to reset world`);
    const { resp: controlResp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/ControlWorld',
        serviceName: `/world/${world}/control`,
        payload: { world_control: { reset: { all: true }} }
      },
      { timeoutMs }
    );
    
    if (!(controlResp?.success ?? controlResp?.ok ?? controlResp?.boolean)) {
      console.warn(`[${new Date().toISOString()}] [makeResetVisualization] Gazebo control service failed: ${controlResp?.message || 'Unknown error'}`);
    } else {
      console.log(`[${new Date().toISOString()}] [makeResetVisualization] Gazebo control service succeeded`);
    }
    
    // If visualization is enabled, refresh it using makeSetVisualization
    if (vizState) {
      console.log(`[${new Date().toISOString()}] [makeResetVisualization] Visualization is enabled, refreshing visualization`);
      const setVisualization = makeSetVisualization(node, { timeoutMs });
      await setVisualization({ value: async () => true });
      console.log(`[${new Date().toISOString()}] [makeResetVisualization] Visualization refreshed successfully`);
    }
    
    console.log(`[${new Date().toISOString()}] [makeResetVisualization] Visualization reset completed successfully`);
    return `Visualization reset completed. Called Gazebo reset${vizState ? ', visualization refreshed' : ''}.`;
    
  } catch (error) {
    console.warn(`[${new Date().toISOString()}] [makeResetVisualization] Visualization reset failed: ${error.message}, continuing with simulation reset`);
    // Don't throw error, just log warning and continue
  }
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

    const world = get_world();
    if (!world) throw new Error('No active world found');

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
      
      // Start image bridge using bridge manager
      if (world) {
        await setupBridge(node, 'image_bridge', world, { timeoutMs });
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
      
      // Stop image bridge using bridge manager
      await stopBridge(node, 'image_bridge', { timeoutMs });
      
      // Stop web video server
      await stopWebVideoServer(node, { timeoutMs });
      
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

      // Check for file conflicts before saving
      const hasConflict = await checkFileConflict(safeName);
      if (hasConflict) {
        throw new Error(`World file '${safeName}' already exists. Please choose a different name.`);
      }

      const filePath = path.join('/project-root', 'Assets', 'gazebo', 'worlds', 'saved', safeName);
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
 * Input: { fileName: string, autoRun: boolean, headless: boolean }
 */
function makeLaunchSimulation(node, { timeoutMs = 1000 } = {}) {
  return async function launchSimulationAction(input) {
    const data = await input.value();
    const fileName = data?.fileName;
    const autoRun = data?.autoRun !== undefined ? data.autoRun : true;
    const headless = data?.headless !== undefined ? data.headless : true;

    if (!fileName) throw new Error("Missing fileName for launchSimulation");

    const fullPath = await resolveFilePath(fileName);
    if (!fullPath) throw new Error(`File "${fileName}" not found in Assets or upload folders`);

    if (simProcessName) throw new Error("Simulation already running");

    // Auto-detect method based on file extension
    const fileExtension = path.extname(fileName).toLowerCase();
    const method = fileExtension === '.py' ? 'ros2' : 'gz';
    
    const cmd = method === "gz" 
      ? "gz sim" 
      : "ros2 launch";
    
    // Build command arguments based on boolean properties
    const cmdArgs = [fullPath];
    if (method === "gz") {
      if (autoRun) cmdArgs.push("-r");
      if (headless) cmdArgs.push("-s", "--headless-rendering");
    } else if (method === "ros2") {
      // For ROS2 launch files, pass arguments to control GUI tools
      if (headless) {
        cmdArgs.push("rqt:=false", "rviz:=false", "gui:=false", "use_sim_time:=true");
      }
    }

    const fullCmd = `${cmd} ${cmdArgs.join(" ")}`;
    const processName = "gz_sim"; // Unique process name

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
          // Extract world name from the file (SDF or launch file)
          let worldName;
          let sdfPath = fullPath;
          
          if (method === 'ros2') {
            // For ROS2 launch files, first extract the SDF path
            console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] Extracting SDF path from ROS2 launch file`);
            const extractedSdfPath = await extractSdfFromLaunch(fullPath);
            if (extractedSdfPath) {
              sdfPath = extractedSdfPath;
              console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] Using extracted SDF path: ${sdfPath}`);
            } else {
              console.warn(`[${new Date().toISOString()}] [makeLaunchSimulation] Could not extract SDF path from launch file, using original path`);
            }
          }
          
          // Extract world name from the SDF file
          worldName = await extract_world(sdfPath);
          console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] World name extracted: ${worldName}`);
          
          // Check if the world file contains UR10 robot references
          const hasUr10 = await ur10Exists(sdfPath);
          console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] UR10 detected in world: ${hasUr10}`);
          
          // Set up bridges after simulation starts (excluding image_bridge which is managed by visualization)
          await setupBridges(node, worldName, ['world_services', 'ros_gz_bridge_addon', 'clock_bridge'], { timeoutMs });
          
          // Set up topic subscriptions after bridges are set up
          try {
            console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] Setting up topic subscriptions...`);
            const subscriptions = await setupAllObservableProperties(node);
            topicSubscriptions = subscriptions;
            if (subscriptions.length > 0) {
              console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] Topic subscriptions set up successfully: ${subscriptions.length} subscriptions`);
            } else {
              console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] No topic subscriptions created (no active world detected)`);
            }
          } catch (subErr) {
            console.warn(`[${new Date().toISOString()}] [makeLaunchSimulation] Topic subscription setup failed:`, subErr.message);
          }
          
          // Start UR10 configuration if UR10 is detected in the world
          let ur10Message = '';
          if (hasUr10) {
            console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] Starting UR10 configuration...`);
            try {
              // Wait a bit for the simulation to fully initialize before starting UR10
              await new Promise(resolve => setTimeout(resolve, 3000));
              
              // Start UR10 configuration with a default entity name and wait for completion
              const ur10Result = await startUr10Configuration(node, 'bot', { timeoutMs: 60000 });
              
              if (ur10Result.success) {
                ur10Message = ` UR10 configuration completed successfully`;
                console.log(`[${new Date().toISOString()}] [makeLaunchSimulation] UR10 configuration completed successfully`);
              } else {
                const failedProcesses = [];
                if (ur10Result.results.config.error) {
                  failedProcesses.push(`config: ${ur10Result.results.config.error}`);
                }
                if (ur10Result.results.controller.error) {
                  failedProcesses.push(`controller: ${ur10Result.results.controller.error}`);
                }
                ur10Message = ` (UR10 processes failed: ${failedProcesses.join(', ')})`;
                console.warn(`[${new Date().toISOString()}] [makeLaunchSimulation] UR10 configuration failed:`, failedProcesses);
              }
            } catch (ur10Err) {
              ur10Message = ` (UR10 configuration error: ${ur10Err.message})`;
              console.error(`[${new Date().toISOString()}] [makeLaunchSimulation] UR10 configuration error:`, ur10Err.message);
            }
          }
          
          return `Simulation launched: ${fullPath}, world: ${worldName}${ur10Message}`;
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
        
        // Clean up topic subscriptions
        if (topicSubscriptions.length > 0) {
          try {
            console.log(`[${new Date().toISOString()}] [makeExitSimulation] Cleaning up ${topicSubscriptions.length} topic subscriptions...`);
            cleanupSubscriptions(topicSubscriptions);
            topicSubscriptions = [];
            console.log(`[${new Date().toISOString()}] [makeExitSimulation] Topic subscriptions cleaned up successfully`);
          } catch (cleanupErr) {
            console.warn(`[${new Date().toISOString()}] [makeExitSimulation] Topic subscription cleanup failed:`, cleanupErr.message);
          }
        }
        
        // Reset simulation process name
        simProcessName = null;
        
        // Clean up all bridges (including image_bridge when simulation exits)
        await stopAllBridges(node, { timeoutMs });
        
        // Clean up web video server
        await stopWebVideoServer(node, { timeoutMs });
        
        // Clean up all UR10 processes
        const ur10CleanupResult = await stopAllUr10Configurations(node, { timeoutMs });
        if (ur10CleanupResult.stoppedCount > 0) {
          console.log(`[${new Date().toISOString()}] [makeExitSimulation] Cleaned up ${ur10CleanupResult.stoppedCount} UR10 processes`);
        }
        
        
        // Reset state
        vizState = false;
        clear_world();
        
        return `Simulation exited successfully. Process: ${simProcessName}`;
      } else {
        console.warn(`[${new Date().toISOString()}] [makeExitSimulation] Failed to stop simulation process: ${resp?.message || 'Unknown error'}`);
        throw new Error(`Failed to stop simulation: ${resp?.message || 'Unknown error'}`);
      }
    } catch (error) {
      console.error(`[${new Date().toISOString()}] [makeExitSimulation] Error stopping simulation process: ${error.message}`);
      
      // Fallback cleanup in case of error
      simProcessName = null;
      vizState = false;
      ur10Processes.clear(); // Clear UR10 processes in fallback cleanup
      
      
      clear_world();
    }

    console.log(`[${new Date().toISOString()}] [makeExitSimulation] Simulation exit completed`);
    return "Simulation exited.";
  };
}

// ============================================================================
// UR10 HELPER FUNCTIONS
// ============================================================================

/**
 * Check if a file name indicates UR10 entity
 */
function isUr10Entity(fileName) {
  return fileName === 'ur10_rg2.urdf';
}

/**
 * Generate UR10 process names for an entity
 */
function generateUr10ProcessNames(entityName) {
  return {
    config: `ur10_config_${entityName}`,
    controller: `ur10_controller_${entityName}`
  };
}

/**
 * Get UR10 process commands
 */
function getUr10ProcessCommands() {
  return {
    config: 'bash /project-root/Assets/urdf/robots/ur10_rg2/ur10_config.sh',
    controller: 'node /project-root/bot_servers/ur10_controller.js'
  };
}

/**
 * Wait for MoveIt to be ready (using timeout-based approach to avoid triggering planning requests)
 */
async function isMoveItReady(node, { timeoutMs = 30000 } = {}) {
  // Based on the logs, MoveIt typically takes about 10-15 seconds to fully initialize
  // We'll wait a fixed amount of time to avoid calling any services that might trigger planning
  const waitTime = Math.min(15000, timeoutMs); // Wait up to 15 seconds or the provided timeout
  
  console.log(`[${new Date().toISOString()}] [isMoveItReady] Waiting ${waitTime}ms for MoveIt to initialize...`);
  
  await new Promise(resolve => setTimeout(resolve, waitTime));
  
  console.log(`[${new Date().toISOString()}] [isMoveItReady] MoveIt initialization wait completed`);
  return true;
}

/**
 * Wait for UR10 controller to be ready by checking for MoveIt services
 */
async function waitForUr10ControllerReady(node, { timeoutMs = 30000, checkIntervalMs = 2000 } = {}) {
  console.log(`[${new Date().toISOString()}] [waitForUr10ControllerReady] Waiting for UR10 controller to be ready...`);
  
  const startTime = Date.now();
  const timeout = timeoutMs;
  
  while (Date.now() - startTime < timeout) {
    try {
      // Check if MoveIt services are available - this indicates the UR10 system is ready
      const { resp } = await callService(
        node,
        {
          srvType: 'moveit_msgs/srv/GetPlanningScene',
          serviceName: '/get_planning_scene',
          payload: { components: { components: [] } }
        },
        { timeoutMs: 1000 }
      );
      
      if (resp) {
        console.log(`[${new Date().toISOString()}] [waitForUr10ControllerReady] MoveIt planning scene service is available, UR10 is ready`);
        return true;
      }
      
    } catch (error) {
      // Service not ready yet, continue waiting
    }
    
    // Also try checking for the move_group service as an alternative
    try {
      const { resp } = await callService(
        node,
        {
          srvType: 'moveit_msgs/srv/GetPositionIK',
          serviceName: '/compute_ik',
          payload: { 
            ik_request: {
              group_name: 'ur_manipulator',
              robot_state: { joint_state: { name: [], position: [] } },
              pose_stamped: {
                header: { frame_id: 'base_link' },
                pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }
              }
            }
          }
        },
        { timeoutMs: 1000 }
      );
      
      if (resp) {
        console.log(`[${new Date().toISOString()}] [waitForUr10ControllerReady] MoveIt IK service is available, UR10 is ready`);
        return true;
      }
      
    } catch (error) {
      // Service not ready yet, continue waiting
    }
    
    console.log(`[${new Date().toISOString()}] [waitForUr10ControllerReady] MoveIt services not ready yet, waiting ${checkIntervalMs}ms...`);
    await new Promise(resolve => setTimeout(resolve, checkIntervalMs));
  }
  
  console.warn(`[${new Date().toISOString()}] [waitForUr10ControllerReady] Timeout reached, MoveIt services may not be fully ready`);
  return false;
}

/**
 * Start a single UR10 process (config or controller)
 */
async function startUr10Process(node, processName, command, { timeoutMs = 1000 } = {}) {
  try {
    const { resp } = await callService(
      node,
      {
        srvType: 'sim_process_supervisor_interfaces/srv/ManagedStart',
        serviceName: '/process/managed/start',
        payload: {
          name: processName,
          cmd: command
        }
      },
      { timeoutMs }
    );
    
    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      console.log(`[${new Date().toISOString()}] [startUr10Process] Process started successfully: ${processName}`);
      return { success: true, processName };
    } else {
      console.warn(`[${new Date().toISOString()}] [startUr10Process] Failed to start process: ${processName} - ${resp?.message || 'Unknown error'}`);
      return { success: false, error: resp?.message || 'Unknown error' };
    }
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [startUr10Process] Error starting process ${processName}:`, error.message);
    return { success: false, error: error.message };
  }
}

/**
 * Start UR10 controller as a child process
 */
function startUr10ControllerChildProcess(entityName) {
  const controllerPath = path.join(__dirname, '../../bot_servers/ur10_controller.js');
  
  console.log(`[${new Date().toISOString()}] [startUr10ControllerChildProcess] Starting UR10 controller child process for ${entityName}`);
  console.log(`[${new Date().toISOString()}] [startUr10ControllerChildProcess] Controller path: ${controllerPath}`);
  
  try {
    const childProcess = spawn('node', [controllerPath], {
      stdio: ['ignore', 'pipe', 'pipe'],
      detached: false
    });
    
    // Handle process events
    childProcess.on('error', (error) => {
      console.error(`[${new Date().toISOString()}] [startUr10ControllerChildProcess] Child process error for ${entityName}:`, error.message);
    });
    
    childProcess.on('exit', (code, signal) => {
      console.log(`[${new Date().toISOString()}] [startUr10ControllerChildProcess] Child process exited for ${entityName}: code=${code}, signal=${signal}`);
    });
    
    // Log stdout
    childProcess.stdout.on('data', (data) => {
      console.log(`[${new Date().toISOString()}] [ur10_controller_${entityName}:stdout] ${data.toString().trim()}`);
    });
    
    // Log stderr
    childProcess.stderr.on('data', (data) => {
      console.error(`[${new Date().toISOString()}] [ur10_controller_${entityName}:stderr] ${data.toString().trim()}`);
    });
    
    console.log(`[${new Date().toISOString()}] [startUr10ControllerChildProcess] UR10 controller child process started successfully for ${entityName}, PID: ${childProcess.pid}`);
    return { success: true, childProcess };
    
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [startUr10ControllerChildProcess] Error starting child process for ${entityName}:`, error.message);
    return { success: false, error: error.message };
  }
}

/**
 * Stop a single UR10 process
 */
async function stopUr10Process(node, processName, { timeoutMs = 1000 } = {}) {
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
      console.log(`[${new Date().toISOString()}] [stopUr10Process] Process stopped successfully: ${processName}`);
      return { success: true, processName };
    } else {
      console.warn(`[${new Date().toISOString()}] [stopUr10Process] Failed to stop process: ${processName} - ${resp?.message || 'Unknown error'}`);
      return { success: false, error: resp?.message || 'Unknown error' };
    }
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [stopUr10Process] Error stopping process ${processName}:`, error.message);
    return { success: false, error: error.message };
  }
}

/**
 * Start all UR10 processes for an entity (sequential startup: config first, then controller after MoveIt is ready)
 */
async function startUr10Configuration(node, entityName, { timeoutMs = 1000 } = {}) {
  const processNames = generateUr10ProcessNames(entityName);
  const commands = getUr10ProcessCommands();
  
  console.log(`[${new Date().toISOString()}] [startUr10Configuration] Starting UR10 processes for ${entityName}`);
  console.log(`[${new Date().toISOString()}] [startUr10Configuration] Config process: ${processNames.config}`);
  console.log(`[${new Date().toISOString()}] [startUr10Configuration] Controller process: ${processNames.controller}`);
  
  const results = {
    config: { success: false, processName: processNames.config },
    controller: { success: false, processName: processNames.controller }
  };
  
  // Step 1: Start configuration process
  console.log(`[${new Date().toISOString()}] [startUr10Configuration] Step 1: Starting configuration process...`);
  results.config = await startUr10Process(node, processNames.config, commands.config, { timeoutMs });
  
  if (!results.config.success) {
    console.error(`[${new Date().toISOString()}] [startUr10Configuration] Configuration process failed, skipping controller startup`);
    return { 
      success: false, 
      results,
      processNames
    };
  }
  
  // Step 2: Wait for MoveIt to be ready
  console.log(`[${new Date().toISOString()}] [startUr10Configuration] Step 2: Waiting for MoveIt to be ready...`);
  const moveItReady = await isMoveItReady(node, { timeoutMs: 60000, checkIntervalMs: 3000 });
  
  if (!moveItReady) {
    console.warn(`[${new Date().toISOString()}] [startUr10Configuration] MoveIt not ready within timeout, starting controller anyway...`);
  }
  
  // Step 3: Start controller process as child process
  console.log(`[${new Date().toISOString()}] [startUr10Configuration] Step 3: Starting controller process as child process...`);
  const controllerResult = startUr10ControllerChildProcess(entityName);
  results.controller = controllerResult;
  
  // Step 4: Wait for controller to be ready (only if controller started successfully)
  if (results.controller.success) {
    console.log(`[${new Date().toISOString()}] [startUr10Configuration] Step 4: Waiting for controller to be ready...`);
    const controllerReady = await waitForUr10ControllerReady(node, { timeoutMs: 30000 });
    
    if (controllerReady) {
      console.log(`[${new Date().toISOString()}] [startUr10Configuration] Controller is ready and active`);
    } else {
      console.warn(`[${new Date().toISOString()}] [startUr10Configuration] Controller may not be fully ready, but continuing...`);
    }
  }
  
  // Store process information if at least one succeeded
  if (results.config.success || results.controller.success) {
    ur10Processes.set(entityName, {
      config: results.config.success ? processNames.config : null,
      controller: results.controller.success ? results.controller.childProcess : null
    });
  }
  
  const overallSuccess = results.config.success || results.controller.success;
  return { 
    success: overallSuccess, 
    results,
    processNames
  };
}

/**
 * Stop all UR10 processes for an entity
 */
async function stopUr10Configuration(node, entityName, { timeoutMs = 1000 } = {}) {
  if (!ur10Processes.has(entityName)) {
    console.log(`[${new Date().toISOString()}] [stopUr10Configuration] No UR10 processes found for entity: ${entityName}`);
    return { success: true, message: 'No UR10 processes found' };
  }
  
  const entityProcesses = ur10Processes.get(entityName);
  console.log(`[${new Date().toISOString()}] [stopUr10Configuration] Stopping UR10 processes for ${entityName}`);
  
  const results = {
    config: entityProcesses.config ? await stopUr10Process(node, entityProcesses.config, { timeoutMs }) : { success: true, message: 'No config process' },
    controller: { success: false, processName: 'child_process' }
  };
  
  // Stop controller child process if it exists
  if (entityProcesses.controller) {
    try {
      // Kill the child process
      entityProcesses.controller.kill('SIGTERM');
      results.controller.success = true;
      console.log(`[${new Date().toISOString()}] [stopUr10Configuration] UR10 controller child process stopped successfully, PID: ${entityProcesses.controller.pid}`);
    } catch (error) {
      console.error(`[${new Date().toISOString()}] [stopUr10Configuration] Error stopping UR10 controller child process:`, error.message);
      results.controller.error = error.message;
    }
  } else {
    results.controller = { success: true, message: 'No controller process' };
  }
  
  // Remove from tracking
  ur10Processes.delete(entityName);
  
  const overallSuccess = results.config.success || results.controller.success;
  return { 
    success: overallSuccess, 
    results,
    processNames: {
      config: entityProcesses.config,
      controller: entityProcesses.controller
    }
  };
}

/**
 * Stop all UR10 processes across all entities
 */
async function stopAllUr10Configurations(node, { timeoutMs = 1000 } = {}) {
  if (ur10Processes.size === 0) {
    console.log(`[${new Date().toISOString()}] [stopAllUr10Configurations] No UR10 processes to stop`);
    return { success: true, stoppedCount: 0 };
  }
  
  console.log(`[${new Date().toISOString()}] [stopAllUr10Configurations] Stopping UR10 processes for ${ur10Processes.size} entities...`);
  let stoppedCount = 0;
  const errors = [];
  
  for (const [entityName, entityProcesses] of ur10Processes) {
    // Stop configuration process if it exists
    if (entityProcesses.config) {
      const result = await stopUr10Process(node, entityProcesses.config, { timeoutMs });
      if (result.success) {
        stoppedCount++;
      } else {
        errors.push(`Failed to stop config ${entityProcesses.config}: ${result.error}`);
      }
    }
    
    // Stop controller child process if it exists
    if (entityProcesses.controller) {
      try {
        entityProcesses.controller.kill('SIGTERM');
        stoppedCount++;
        console.log(`[${new Date().toISOString()}] [stopAllUr10Configurations] UR10 controller child process stopped, PID: ${entityProcesses.controller.pid}`);
      } catch (error) {
        errors.push(`Failed to stop controller child process: ${error.message}`);
      }
    }
  }
  
  ur10Processes.clear();
  console.log(`[${new Date().toISOString()}] [stopAllUr10Configurations] Stopped ${stoppedCount} UR10 processes`);
  
  return { 
    success: errors.length === 0, 
    stoppedCount, 
    errors: errors.length > 0 ? errors : undefined 
  };
}

/**
 * Handle UR10 processes for entity spawning
 */
async function handleUr10Spawning(node, entityName, fileName, { timeoutMs = 1000 } = {}) {
  if (!isUr10Entity(fileName)) {
    return null;
  }
  
  const ur10Result = await startUr10Configuration(node, entityName, { timeoutMs });
  
  if (ur10Result.success) {
    return ` UR10 configuration completed successfully`;
  } else {
    const failedProcesses = [];
    if (ur10Result.results.config.error) {
      failedProcesses.push(`config: ${ur10Result.results.config.error}`);
    }
    if (ur10Result.results.controller.error) {
      failedProcesses.push(`controller: ${ur10Result.results.controller.error}`);
    }
    return ` (UR10 processes failed: ${failedProcesses.join(', ')})`;
  }
}

/**
 * Handle UR10 processes for entity deletion
 */
async function handleUr10Deletion(node, entityName, { timeoutMs = 1000 } = {}) {
  if (!ur10Processes.has(entityName)) {
    return null;
  }
  
  const ur10Result = await stopUr10Configuration(node, entityName, { timeoutMs });
  
  if (ur10Result.success) {
    const stoppedProcesses = [];
    if (ur10Result.results.config.success) {
      stoppedProcesses.push(`config: ${ur10Result.results.config.processName}`);
    }
    if (ur10Result.results.controller.success) {
      stoppedProcesses.push(`controller: ${ur10Result.results.controller.processName}`);
    }
    if (stoppedProcesses.length > 0) {
      return ` UR10 processes stopped (${stoppedProcesses.join(', ')})`;
    }
  } else {
    const failedProcesses = [];
    if (ur10Result.results.config.error) {
      failedProcesses.push(`config: ${ur10Result.results.config.error}`);
    }
    if (ur10Result.results.controller.error) {
      failedProcesses.push(`controller: ${ur10Result.results.controller.error}`);
    }
    if (failedProcesses.length > 0) {
      return ` (UR10 processes stop failed: ${failedProcesses.join(', ')})`;
    }
  }
  
  return null;
}

module.exports = {
  makeSetRtf,
  makeDeleteEntity,
  makeSetEntityPose,
  makeSpawnEntity,
  makeSimControl,
  makeResetVisualization,
  makeSetVisualization,
  makeSaveWorld,
  makeLaunchSimulation,
  makeExitSimulation,
  visualizationRead,
};
