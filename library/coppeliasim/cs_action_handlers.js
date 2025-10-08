/**
 * @fileoverview CoppeliaSim ROS 2 Service Integration Library
 * 
 * This module provides Web of Things (WoT) action handlers for CoppeliaSim simulation control
 * via ROS 2 services. It acts as a bridge between WoT interfaces and CoppeliaSim's ROS 2 API.
 * 
 * Key Features:
 * - Simulation control (start, pause, stop, speed adjustment)
 * - Direct integration with CoppeliaSim ROS 2 services
 * - Scene and model management
 * - Synchronized simulation mode support
 * 
 * Architecture:
 * - Factory pattern for action handlers that capture ROS 2 node instances
 * - Service-based communication with CoppeliaSim via ROS 2
 * - All services under /coppeliasim namespace
 * 
 * @author Yifan & Cursor & ChatGPT
 * @version 1.0.0
 */

const { callService } = require('../common/ros2_service_helper');
const { resolveFilePath, checkFileConflict } = require('../common/fileUtils');
const { deg2quat } = require('../common/deg2quat');
const fs = require('fs').promises;
const path = require('path');
const { spawn } = require('child_process'); // For UR10 child process management

// Track UR10 processes by model handle: { moveit: childProcess, wotServer: childProcess }
let ur10Processes = new Map();

/**
 * Map CoppeliaSim speed factor (-3 to 6) to actual speed multiplier
 * @param {number} speedFactor - Internal speed factor from CoppeliaSim
 * @returns {number} Actual speed multiplier
 */
function speedFactorToMultiplier(speedFactor) {
  const mapping = {
    '-3': 0.1,
    '-2': 0.2,
    '-1': 0.5,
    '0': 1,
    '1': 2,
    '2': 4,
    '3': 8,
    '4': 16,
    '5': 32,
    '6': 64
  };
  return mapping[String(speedFactor)] ?? 1;
}

/**
 * Check if a file is a UR10 RG2 model
 */
function isUR10Model(fileName) {
  return fileName && fileName.includes('ur10_rg2_coppelia.urdf');
}

/**
 * Spawn MoveIt stack child process for UR10 RG2
 */
function spawnMoveItStack(modelHandle) {
  console.log(`[${new Date().toISOString()}] [spawnMoveItStack] Starting MoveIt stack for handle ${modelHandle}...`);
  
  const script = '/project-root/library/coppeliasim/start_moveit.sh';
  const moveit = spawn('bash', [script], {
    stdio: ['ignore', 'pipe', 'pipe'],
    detached: false
  });
  
  moveit.stdout.on('data', (data) => {
    console.log(`[${new Date().toISOString()}] [moveit_${modelHandle}:stdout] ${data.toString().trim()}`);
  });
  
  moveit.stderr.on('data', (data) => {
    console.error(`[${new Date().toISOString()}] [moveit_${modelHandle}:stderr] ${data.toString().trim()}`);
  });
  
  moveit.on('error', (error) => {
    console.error(`[${new Date().toISOString()}] [spawnMoveItStack] Error for handle ${modelHandle}:`, error.message);
  });
  
  moveit.on('exit', (code, signal) => {
    console.log(`[${new Date().toISOString()}] [spawnMoveItStack] MoveIt stack exited for handle ${modelHandle}: code=${code}, signal=${signal}`);
  });
  
  console.log(`[${new Date().toISOString()}] [spawnMoveItStack] MoveIt stack started, PID: ${moveit.pid}`);
  return moveit;
}

/**
 * Spawn WoT server child process for UR10 RG2
 */
function spawnWoTServer(modelHandle) {
  console.log(`[${new Date().toISOString()}] [spawnWoTServer] Starting WoT server for handle ${modelHandle}...`);
  
  const serverPath = '/project-root/Assets/urdf/examples/robots/ur10_rg2/ur10_server_coppelia.js';
  const wotServer = spawn('node', [serverPath], {
    stdio: ['ignore', 'pipe', 'pipe'],
    detached: false
  });
  
  wotServer.stdout.on('data', (data) => {
    console.log(`[${new Date().toISOString()}] [wot_server_${modelHandle}:stdout] ${data.toString().trim()}`);
  });
  
  wotServer.stderr.on('data', (data) => {
    console.error(`[${new Date().toISOString()}] [wot_server_${modelHandle}:stderr] ${data.toString().trim()}`);
  });
  
  wotServer.on('error', (error) => {
    console.error(`[${new Date().toISOString()}] [spawnWoTServer] Error for handle ${modelHandle}:`, error.message);
  });
  
  wotServer.on('exit', (code, signal) => {
    console.log(`[${new Date().toISOString()}] [spawnWoTServer] WoT server exited for handle ${modelHandle}: code=${code}, signal=${signal}`);
  });
  
  console.log(`[${new Date().toISOString()}] [spawnWoTServer] WoT server started, PID: ${wotServer.pid}`);
  return wotServer;
}

/**
 * Stop UR10 processes for a specific model
 */
function stopUR10Processes(modelHandle) {
  if (!ur10Processes.has(modelHandle)) {
    console.log(`[${new Date().toISOString()}] [stopUR10Processes] No processes found for handle ${modelHandle}`);
    return { success: true, message: 'No processes found' };
  }
  
  const processes = ur10Processes.get(modelHandle);
  console.log(`[${new Date().toISOString()}] [stopUR10Processes] Stopping processes for handle ${modelHandle}...`);
  
  const results = { moveit: false, wotServer: false };
  
  // Stop MoveIt stack
  if (processes.moveit) {
    try {
      processes.moveit.kill('SIGTERM');
      results.moveit = true;
      console.log(`[${new Date().toISOString()}] [stopUR10Processes] MoveIt stack stopped, PID: ${processes.moveit.pid}`);
    } catch (error) {
      console.error(`[${new Date().toISOString()}] [stopUR10Processes] Error stopping MoveIt:`, error.message);
    }
  }
  
  // Stop WoT server
  if (processes.wotServer) {
    try {
      processes.wotServer.kill('SIGTERM');
      results.wotServer = true;
      console.log(`[${new Date().toISOString()}] [stopUR10Processes] WoT server stopped, PID: ${processes.wotServer.pid}`);
    } catch (error) {
      console.error(`[${new Date().toISOString()}] [stopUR10Processes] Error stopping WoT server:`, error.message);
    }
  }
  
  ur10Processes.delete(modelHandle);
  return { success: results.moveit || results.wotServer, results };
}

/**
 * Stop all UR10 processes
 */
function stopAllUR10Processes() {
  if (ur10Processes.size === 0) {
    console.log(`[${new Date().toISOString()}] [stopAllUR10Processes] No processes to stop`);
    return { success: true, stoppedCount: 0 };
  }
  
  console.log(`[${new Date().toISOString()}] [stopAllUR10Processes] Stopping ${ur10Processes.size} UR10 instances...`);
  let stoppedCount = 0;
  
  for (const [handle, processes] of ur10Processes) {
    const result = stopUR10Processes(handle);
    if (result.success) stoppedCount++;
  }
  
  ur10Processes.clear();
  console.log(`[${new Date().toISOString()}] [stopAllUR10Processes] Stopped ${stoppedCount} UR10 instances`);
  
  return { success: true, stoppedCount };
}

/**
 * Create simControl action handler for CoppeliaSim
 * Maps WoT modes to CoppeliaSim ROS2 services:
 * - run -> /coppeliasim/start
 * - pause -> /coppeliasim/pause  
 * - stop -> /coppeliasim/stop
 * - faster -> /coppeliasim/faster
 * - slower -> /coppeliasim/slower
 * 
 * @param {Object} node - ROS 2 node instance
 * @param {Object} options - Configuration options
 * @param {number} options.timeoutMs - Service call timeout in milliseconds (default: 1000)
 * @returns {Function} WoT action handler function
 */
function makeSimControl(node, { timeoutMs = 1000 } = {}) {
  return async function simControlAction(input) {
    const data = await input.value();
    const mode = data?.mode;

    if (!mode) {
      throw new Error("Missing mode for simControl");
    }

    let serviceName, serviceType;
    
    switch (mode) {
      case "run":
        serviceName = "/coppeliasim/start";
        serviceType = "std_srvs/srv/Trigger";
        break;
      case "pause":
        serviceName = "/coppeliasim/pause";
        serviceType = "std_srvs/srv/Trigger";
        break;
      case "stop":
        serviceName = "/coppeliasim/stop";
        serviceType = "std_srvs/srv/Trigger";
        break;
      case "faster":
        serviceName = "/coppeliasim/faster";
        serviceType = "std_srvs/srv/Trigger";
        break;
      case "slower":
        serviceName = "/coppeliasim/slower";
        serviceType = "std_srvs/srv/Trigger";
        break;
      default:
        throw new Error(`Invalid mode: ${mode}. Valid modes are: run, pause, stop, faster, slower`);
    }

    try {
      console.log(`[${new Date().toISOString()}] [simControl] Calling ${serviceName} for mode: ${mode}`);
      
      const { resp } = await callService(
        node,
        {
          srvType: serviceType,
          serviceName: serviceName,
          payload: {} // std_srvs/srv/Trigger has no request fields
        },
        { timeoutMs }
      );

      // std_srvs/srv/Trigger response has success (bool) and message (string) fields
      const success = resp?.success ?? false;
      let message = resp?.message ?? "No message from CoppeliaSim";

      // For faster/slower commands, parse JSON message and translate speed factor
      if ((mode === 'faster' || mode === 'slower') && message) {
        try {
          const parsed = JSON.parse(message);
          if (parsed.speedFactor !== undefined) {
            const speedMultiplier = speedFactorToMultiplier(parsed.speedFactor);
            
            if (success) {
              // Success: show current speed
              message = `Current speed: ×${speedMultiplier}`;
            } else {
              // Failure: already at min/max
              if (mode === 'faster') {
                message = `Already at maximum speed: ×${speedMultiplier}`;
              } else {
                message = `Already at minimum speed: ×${speedMultiplier}`;
              }
            }
          }
        } catch (e) {
          // If parsing fails, keep original message
          console.warn(`[${new Date().toISOString()}] [simControl] Failed to parse message JSON:`, e.message);
        }
      }

      console.log(`[${new Date().toISOString()}] [simControl] Response: success=${success}, message=${JSON.stringify(message)}`);

      return {
        success: success,
        message: message
      };

    } catch (error) {
      console.error(`[${new Date().toISOString()}] [simControl] Error calling ${serviceName}:`, error.message);
      return {
        success: false,
        message: `Failed to ${mode} simulation: ${error.message}`
      };
    }
  };
}

/**
 * Create manageScene action handler for CoppeliaSim
 * Manages scene operations via ROS2 topic publishing:
 * - load -> Load a scene file
 * - close -> Close current scene
 * - save -> Save current scene to file
 * 
 * @param {Object} node - ROS 2 node instance
 * @param {Object} options - Configuration options
 * @param {number} options.timeoutMs - Response timeout in milliseconds (default: 5000)
 * @returns {Function} WoT action handler function
 */
function makeManageScene(node, { timeoutMs = 5000 } = {}) {
  return async function manageSceneAction(input) {
    const data = await input.value();
    const mode = data?.mode;
    const fileName = data?.fileName;

    if (!mode) {
      throw new Error("Missing mode for manageScene");
    }

    // Validate fileName for load and save operations
    if ((mode === 'load' || mode === 'save') && !fileName) {
      throw new Error(`fileName is required for ${mode} operation`);
    }

    // For load operation, resolve the file path
    let resolvedPath = fileName;
    if (mode === 'load' && fileName) {
      try {
        resolvedPath = await resolveFilePath(fileName);
        if (!resolvedPath) {
          return {
            success: false,
            message: `Scene file '${fileName}' not found in Assets directory. Please check the filename and try again.`
          };
        }
        console.log(`[${new Date().toISOString()}] [manageScene] Resolved '${fileName}' to '${resolvedPath}'`);
      } catch (error) {
        return {
          success: false,
          message: `Error resolving file path: ${error.message}`
        };
      }
    }

    // For save operation, construct full path and check if file already exists
    if (mode === 'save' && fileName) {
      // Always use default saved directory
      resolvedPath = path.join('/project-root/Assets/coppeliasim/scenes/saved', fileName);
      
      // Check for conflicts
      const hasConflict = await checkFileConflict(fileName, 'coppeliasim');
      
      if (hasConflict) {
        return {
          success: false,
          message: `File '${fileName}' already exists in assets. Please choose a different name or delete the existing file first.`
        };
      }
    }

    // For close operation, check if simulation is stopped first
    if (mode === 'close') {
      try {
        // Get simulation state by calling the stats topic
        const statsPromise = new Promise((resolve, reject) => {
          let statsHandled = false;
          const statsSub = node.createSubscription(
            'std_msgs/msg/String',
            '/coppeliasim/stats',
            (msg) => {
              if (statsHandled) return;
              statsHandled = true;
              node.destroySubscription(statsSub);
              
              try {
                const stats = JSON.parse(msg.data);
                resolve(stats);
              } catch (e) {
                reject(new Error('Failed to parse stats'));
              }
            }
          );

          setTimeout(() => {
            if (statsHandled) return;
            statsHandled = true;
            node.destroySubscription(statsSub);
            reject(new Error('Timeout getting simulation state'));
          }, 1000);
        });

        const stats = await statsPromise;
        // simState: 0=stopped, 1=running, 2=paused
        if (stats.simState !== 0) {
          const stateText = stats.simState === 1 ? 'running' : 'paused';
          return {
            success: false,
            message: `Cannot close scene while simulation is ${stateText}. Please stop the simulation first.`
          };
        }
      } catch (error) {
        console.warn(`[${new Date().toISOString()}] [manageScene] Could not check simulation state:`, error.message);
        // Continue anyway - let CoppeliaSim handle it
      }
    }

    // Create command JSON
    const command = {
      operation: mode,
      ...(resolvedPath && { scenePath: resolvedPath })
    };

    try {
      console.log(`[${new Date().toISOString()}] [manageScene] Publishing command:`, command);

      // Create publisher and subscriber for request/response pattern
      const commandPub = node.createPublisher('std_msgs/msg/String', '/coppeliasim/manageScene');
      
      // Set up response listener
      const responsePromise = new Promise((resolve, reject) => {
        let isHandled = false;
        
        const responseSub = node.createSubscription(
          'std_msgs/msg/String',
          '/coppeliasim/manageSceneResponse',
          (msg) => {
            if (isHandled) return; // Prevent multiple responses
            
            try {
              const response = JSON.parse(msg.data);
              isHandled = true;
              clearTimeout(timeoutHandle);
              node.destroySubscription(responseSub);
              resolve(response);
            } catch (e) {
              isHandled = true;
              clearTimeout(timeoutHandle);
              node.destroySubscription(responseSub);
              reject(new Error(`Failed to parse response: ${e.message}`));
            }
          }
        );

        // Set timeout
        const timeoutHandle = setTimeout(() => {
          if (isHandled) return; // Already handled
          isHandled = true;
          node.destroySubscription(responseSub);
          reject(new Error('Response timeout'));
        }, timeoutMs);
      });

      // Publish command
      commandPub.publish({ data: JSON.stringify(command) });

      // Wait for response
      const response = await responsePromise;
      
      console.log(`[${new Date().toISOString()}] [manageScene] Response:`, response);

      // Clean up publisher
      node.destroyPublisher(commandPub);

      return {
        success: response.success ?? false,
        message: response.message ?? "No message from CoppeliaSim"
      };

    } catch (error) {
      console.error(`[${new Date().toISOString()}] [manageScene] Error:`, error.message);
      return {
        success: false,
        message: `Failed to ${mode} scene: ${error.message}`
      };
    }
  };
}

/**
 * Create manageModel action handler for CoppeliaSim
 * Manages model operations via ROS2 topic publishing:
 * - load -> Load a model file (URDF or TTM)
 * - remove -> Remove a model by handle
 * - setPose -> Set pose of a model by handle
 * 
 * @param {Object} node - ROS 2 node instance
 * @param {Object} options - Configuration options
 * @param {number} options.timeoutMs - Response timeout in milliseconds (default: 5000)
 * @returns {Function} WoT action handler function
 */
function makeManageModel(node, { timeoutMs = 5000 } = {}) {
  return async function manageModelAction(input) {
    const data = await input.value();
    const mode = data?.mode;
    const fileName = data?.fileName;
    let handle = data?.handle;
    const name = data?.name;
    const objectName = data?.objectName;
    const position = data?.position;
    const orientation = data?.orientation;

    if (!mode) {
      throw new Error("Missing mode for manageModel");
    }

    // Validate inputs based on mode
    if (mode === 'load' && !fileName) {
      throw new Error("fileName is required for load operation");
    }
    if (mode === 'remove' && !handle && !name) {
      throw new Error("Either handle or name is required for remove operation");
    }
    if (mode === 'setPose' && !handle) {
      throw new Error("handle is required for setPose operation");
    }
    if (mode === 'setPose' && !orientation) {
      throw new Error("orientation is required for setPose operation");
    }

    // For remove operation by name, resolve name to handle
    if (mode === 'remove' && name && !handle) {
      try {
        // Get current models from the models topic
        const modelsPromise = new Promise((resolve, reject) => {
          let modelsHandled = false;
          const modelsSub = node.createSubscription(
            'std_msgs/msg/String',
            '/coppeliasim/models',
            (msg) => {
              if (modelsHandled) return;
              modelsHandled = true;
              node.destroySubscription(modelsSub);
              
              try {
                const models = JSON.parse(msg.data);
                resolve(models);
              } catch (e) {
                reject(new Error('Failed to parse models'));
              }
            }
          );

          setTimeout(() => {
            if (modelsHandled) return;
            modelsHandled = true;
            node.destroySubscription(modelsSub);
            reject(new Error('Timeout getting models list'));
          }, 1000);
        });

        const models = await modelsPromise;
        
        // Find model by name (check both with and without leading slash)
        if (Array.isArray(models)) {
          const model = models.find(m => {
            const cleanName = m.name && m.name.startsWith('/') ? m.name.substring(1) : m.name;
            return cleanName === name || m.name === name;
          });
          
          if (model) {
            handle = model.handle;
            console.log(`[${new Date().toISOString()}] [manageModel] Resolved name '${name}' to handle ${handle}`);
          } else {
            return {
              success: false,
              message: `Object with name '${name}' not found in the scene.`
            };
          }
        }
      } catch (error) {
        console.warn(`[${new Date().toISOString()}] [manageModel] Could not resolve name to handle:`, error.message);
        return {
          success: false,
          message: `Failed to resolve name '${name}' to handle: ${error.message}`
        };
      }
    }

    // For remove operation by handle, verify object exists
    if (mode === 'remove' && handle !== undefined) {
      try {
        // Get current models from the models topic
        const modelsPromise = new Promise((resolve, reject) => {
          let modelsHandled = false;
          const modelsSub = node.createSubscription(
            'std_msgs/msg/String',
            '/coppeliasim/models',
            (msg) => {
              if (modelsHandled) return;
              modelsHandled = true;
              node.destroySubscription(modelsSub);
              
              try {
                const models = JSON.parse(msg.data);
                resolve(models);
              } catch (e) {
                reject(new Error('Failed to parse models'));
              }
            }
          );

          setTimeout(() => {
            if (modelsHandled) return;
            modelsHandled = true;
            node.destroySubscription(modelsSub);
            reject(new Error('Timeout getting models list'));
          }, 1000);
        });

        const models = await modelsPromise;
        // Check if handle exists
        if (Array.isArray(models)) {
          const handleExists = models.some(model => model.handle === handle);
          if (!handleExists) {
            return {
              success: false,
              message: `Object with handle ${handle} does not exist. It may have already been removed.`
            };
          }
        }
      } catch (error) {
        console.warn(`[${new Date().toISOString()}] [manageModel] Could not verify object existence:`, error.message);
        // Continue anyway - let CoppeliaSim handle it
      }
    }

    // For load operation, resolve the file path
    let resolvedPath = fileName;
    if (mode === 'load' && fileName) {
      try {
        resolvedPath = await resolveFilePath(fileName);
        if (!resolvedPath) {
          return {
            success: false,
            message: `Model file '${fileName}' not found in Assets directory. Please check the filename and try again.`
          };
        }
        console.log(`[${new Date().toISOString()}] [manageModel] Resolved '${fileName}' to '${resolvedPath}'`);
      } catch (error) {
        return {
          success: false,
          message: `Error resolving file path: ${error.message}`
        };
      }

      // Check for object name conflict if objectName is provided
      if (objectName) {
        try {
          // Get current models from the models topic
          const modelsPromise = new Promise((resolve, reject) => {
            let modelsHandled = false;
            const modelsSub = node.createSubscription(
              'std_msgs/msg/String',
              '/coppeliasim/models',
              (msg) => {
                if (modelsHandled) return;
                modelsHandled = true;
                node.destroySubscription(modelsSub);
                
                try {
                  const models = JSON.parse(msg.data);
                  resolve(models);
                } catch (e) {
                  reject(new Error('Failed to parse models'));
                }
              }
            );

            setTimeout(() => {
              if (modelsHandled) return;
              modelsHandled = true;
              node.destroySubscription(modelsSub);
              reject(new Error('Timeout getting models list'));
            }, 1000);
          });

          const models = await modelsPromise;
          
          // Check if objectName already exists (check both with and without leading slash)
          if (Array.isArray(models)) {
            const nameExists = models.some(m => {
              const cleanName = m.name && m.name.startsWith('/') ? m.name.substring(1) : m.name;
              return cleanName === objectName || m.name === objectName;
            });
            
            if (nameExists) {
              return {
                success: false,
                message: `Object name '${objectName}' is already in use. Please choose a different name.`
              };
            }
          }
        } catch (error) {
          console.warn(`[${new Date().toISOString()}] [manageModel] Could not check name conflict:`, error.message);
          // Continue anyway - let CoppeliaSim handle it
        }
      }
    }

    // Convert position object to array if provided
    let positionArray = null;
    if (position && typeof position === 'object') {
      positionArray = [
        position.x ?? 0,
        position.y ?? 0,
        position.z ?? 0
      ];
    }

    // Convert orientation from Euler angles (degrees) to quaternion if provided
    let orientationArray = null;
    let poseArray = null;
    
    if (orientation && typeof orientation === 'object') {
      const { qx, qy, qz, qw } = deg2quat({
        roll: orientation.roll ?? 0,
        pitch: orientation.pitch ?? 0,
        yaw: orientation.yaw ?? 0
      });
      orientationArray = [qx, qy, qz, qw];
      
      // For setPose, combine position and orientation into pose array
      if (mode === 'setPose') {
        if (!position || typeof position !== 'object') {
          throw new Error("position is required for setPose operation");
        }
        poseArray = [
          position.x ?? 0,
          position.y ?? 0,
          position.z ?? 0,
          qx, qy, qz, qw
        ];
      }
    }

    // Create command JSON
    const command = {
      operation: mode,
      ...(resolvedPath && { modelPath: resolvedPath }),
      ...(handle !== undefined && { handle: handle }),
      ...(objectName && { objectName: objectName }),
      ...(positionArray && mode === 'load' && { position: positionArray }),
      ...(orientationArray && mode === 'load' && { orientation: orientationArray }),
      ...(poseArray && mode === 'setPose' && { pose: poseArray })
    };

    try {
      console.log(`[${new Date().toISOString()}] [manageModel] Publishing command:`, command);

      // Create publisher and subscriber for request/response pattern
      const commandPub = node.createPublisher('std_msgs/msg/String', '/coppeliasim/manageModel');
      
      // Set up response listener
      const responsePromise = new Promise((resolve, reject) => {
        let isHandled = false;
        
        const responseSub = node.createSubscription(
          'std_msgs/msg/String',
          '/coppeliasim/manageModelResponse',
          (msg) => {
            if (isHandled) return; // Prevent multiple responses
            
            try {
              const response = JSON.parse(msg.data);
              isHandled = true;
              clearTimeout(timeoutHandle);
              node.destroySubscription(responseSub);
              resolve(response);
            } catch (e) {
              isHandled = true;
              clearTimeout(timeoutHandle);
              node.destroySubscription(responseSub);
              reject(new Error(`Failed to parse response: ${e.message}`));
            }
          }
        );

        // Set timeout
        const timeoutHandle = setTimeout(() => {
          if (isHandled) return; // Already handled
          isHandled = true;
          node.destroySubscription(responseSub);
          reject(new Error('Response timeout'));
        }, timeoutMs);
      });

      // Publish command
      commandPub.publish({ data: JSON.stringify(command) });

      // Wait for response
      const response = await responsePromise;
      
      console.log(`[${new Date().toISOString()}] [manageModel] Response:`, response);

      // Clean up publisher
      node.destroyPublisher(commandPub);

      // Handle UR10 process management
      let ur10Message = null;
      if (mode === 'load' && response.success && response.handle && isUR10Model(fileName)) {
        console.log(`[${new Date().toISOString()}] [manageModel] UR10 model detected, spawning processes for handle ${response.handle}...`);
        
        try {
          // Spawn MoveIt stack and WoT server as child processes
          const moveitProcess = spawnMoveItStack(response.handle);
          
          // Wait a moment for MoveIt to initialize
          await new Promise(resolve => setTimeout(resolve, 3000));
          
          const wotServerProcess = spawnWoTServer(response.handle);
          
          // Track the processes
          ur10Processes.set(response.handle, {
            moveit: moveitProcess,
            wotServer: wotServerProcess
          });
          
          ur10Message = ` UR10 processes spawned (MoveIt: PID ${moveitProcess.pid}, WoT Server: PID ${wotServerProcess.pid}, available at http://localhost:8082)`;
        } catch (error) {
          console.error(`[${new Date().toISOString()}] [manageModel] Error spawning UR10 processes:`, error.message);
          ur10Message = ` Warning: UR10 model loaded but processes failed to start: ${error.message}`;
        }
      } else if (mode === 'remove' && response.success && handle) {
        // Stop UR10 processes if this model had them
        if (ur10Processes.has(handle)) {
          console.log(`[${new Date().toISOString()}] [manageModel] Stopping UR10 processes for handle ${handle}...`);
          const stopResult = stopUR10Processes(handle);
          if (stopResult.success) {
            ur10Message = ` UR10 processes stopped.`;
          }
        }
      }

      return {
        success: response.success ?? false,
        message: (response.message ?? "No message from CoppeliaSim") + (ur10Message || ''),
        ...(response.handle !== undefined && { handle: response.handle })
      };

    } catch (error) {
      console.error(`[${new Date().toISOString()}] [manageModel] Error:`, error.message);
      return {
        success: false,
        message: `Failed to ${mode} model: ${error.message}`
      };
    }
  };
}

module.exports = {
  makeSimControl,
  makeManageScene,
  makeManageModel,
  stopAllUR10Processes  // Export for cleanup on controller shutdown
};

