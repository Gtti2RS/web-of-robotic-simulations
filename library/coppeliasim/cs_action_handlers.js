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

module.exports = {
  makeSimControl
};

