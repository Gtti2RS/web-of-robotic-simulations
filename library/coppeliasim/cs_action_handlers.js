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
const fs = require('fs').promises;
const path = require('path');

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

module.exports = {
  makeSimControl,
  makeManageScene
};

