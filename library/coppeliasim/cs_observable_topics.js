/**
 * @fileoverview CoppeliaSim Observable Topics Library
 * 
 * This module provides real-time data streaming from CoppeliaSim simulation via ROS 2 topics,
 * implementing Server-Sent Events (SSE) for Web of Things observable properties.
 * 
 * Key Features:
 * - Real-time simulation statistics streaming (simStats)
 * - Live model list tracking (models)
 * - Live object pose monitoring (poses)
 * - Server-Sent Events (SSE) implementation for web clients
 * - Automatic topic subscription management
 * - Connection lifecycle management with keep-alive
 * 
 * Observable Properties:
 * - simStats: Simulation timing, state, speed factor
 * - models: Available models in the scene
 * - poses: Object positions and orientations
 * 
 * @author Yifan & Cursor
 * @version 1.0.0
 */

const rclnodejs = require("rclnodejs");
const { SSEManager } = require("../common/sse_manager");
const { 
  createROS2TopicSubscription, 
  jsonMessageHandler, 
  cleanupSubscriptions: cleanupSubs,
  createCombinedSSEMiddleware,
  createReadHandlers
} = require("../common/observable_utils");

// Global SSE manager instance
const sseManager = new SSEManager();

// Property configurations for CoppeliaSim topics
const PROPERTIES = {
  simStats: {
    topic: "/coppeliasim/stats",
    messageType: "std_msgs/msg/String",
    initialData: { timestamp: new Date().toISOString(), message: "No stats received yet" },
    ssePath: "/cs_controller/properties/simStats/observable"
  },
  models: {
    topic: "/coppeliasim/models",
    messageType: "std_msgs/msg/String",
    initialData: { timestamp: new Date().toISOString(), message: "No models received yet" },
    ssePath: "/cs_controller/properties/models/observable"
  },
  poses: {
    topic: "/coppeliasim/poses",
    messageType: "std_msgs/msg/String",
    initialData: { timestamp: new Date().toISOString(), message: "No poses received yet" },
    ssePath: "/cs_controller/properties/poses/observable"
  }
};

// Initialize all properties
Object.entries(PROPERTIES).forEach(([name, config]) => {
  sseManager.registerProperty(name, config.initialData);
});

// Models message handler - parses the models list from CoppeliaSim
const modelsMessageHandler = (msg) => {
  try {
    const models = JSON.parse(msg.data);
    
    // Remove leading slash from model names
    const cleanedModels = Array.isArray(models) ? models.map(model => ({
      ...model,
      name: model.name && model.name.startsWith('/') ? model.name.substring(1) : model.name
    })) : models;
    
    return {
      models: cleanedModels,
      totalCount: Array.isArray(cleanedModels) ? cleanedModels.length : 0
    };
  } catch (error) {
    throw new Error(`Failed to parse models data: ${error.message}`);
  }
};

// Poses message handler - parses the poses list from CoppeliaSim
const posesMessageHandler = (msg) => {
  try {
    const poses = JSON.parse(msg.data);
    
    // Transform poses: remove leading slash and convert pose array to position/orientation objects
    const transformedPoses = Array.isArray(poses) ? poses.map(poseData => {
      const cleanName = poseData.name && poseData.name.startsWith('/') ? poseData.name.substring(1) : poseData.name;
      
      // Transform pose array [x,y,z,qx,qy,qz,qw] to position/orientation objects
      let position, orientation;
      if (Array.isArray(poseData.pose) && poseData.pose.length >= 7) {
        position = {
          x: poseData.pose[0],
          y: poseData.pose[1],
          z: poseData.pose[2]
        };
        orientation = {
          x: poseData.pose[3],
          y: poseData.pose[4],
          z: poseData.pose[5],
          w: poseData.pose[6]
        };
      }
      
      return {
        name: cleanName,
        handle: poseData.handle,
        ...(position && { position }),
        ...(orientation && { orientation })
      };
    }) : poses;
    
    return {
      poses: transformedPoses,
      totalCount: Array.isArray(transformedPoses) ? transformedPoses.length : 0
    };
  } catch (error) {
    throw new Error(`Failed to parse poses data: ${error.message}`);
  }
};

// Stats message handler - parses simulation statistics from CoppeliaSim
const statsMessageHandler = (msg) => {
  try {
    const stats = JSON.parse(msg.data);
    
    // Map speed factor (-3 to 6) to speed multiplier (0.1 to 64)
    const speedFactorToMultiplier = (speedFactor) => {
      const mapping = {
        '-3': 0.1, '-2': 0.2, '-1': 0.5, '0': 1,
        '1': 2, '2': 4, '3': 8, '4': 16, '5': 32, '6': 64
      };
      return mapping[String(speedFactor)] ?? 1;
    };
    
    // Map simState (0-2) to readable state
    const simStateToString = (simState) => {
      const mapping = { 0: 'stopped', 1: 'running', 2: 'paused' };
      return mapping[simState] ?? 'unknown';
    };
    
    // Build simplified stats object
    const result = {
      simTime: stats.simTime,
      simState: stats.simState !== undefined ? simStateToString(stats.simState) : 'unknown',
      speedMultiplier: stats.speedFactor !== undefined ? speedFactorToMultiplier(stats.speedFactor) : 1,
      realTime: stats.realTime
    };
    
    return result;
  } catch (error) {
    throw new Error(`Failed to parse stats data: ${error.message}`);
  }
};

// Setup all observable properties with ROS2 subscriptions
async function setupAllObservableProperties(node) {
  const subscriptions = [];
  
  console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Setting up CoppeliaSim observable properties`);
  
  // Map property names to their message handlers
  const handlers = {
    simStats: statsMessageHandler,
    models: modelsMessageHandler,
    poses: posesMessageHandler
  };
  
  Object.entries(PROPERTIES).forEach(([propertyName, config]) => {
    console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Processing property: ${propertyName}, topic: ${config.topic}`);
    
    const messageHandler = handlers[propertyName] || jsonMessageHandler;
    
    const subscriber = createROS2TopicSubscription(
      node, 
      config.topic, 
      config.messageType, 
      propertyName, 
      messageHandler,
      sseManager
    );
    
    if (subscriber) {
      subscriptions.push(subscriber);
      console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Successfully created subscription for ${propertyName}`);
    } else {
      console.warn(`[${new Date().toISOString()}] [setupAllObservableProperties] Failed to create subscription for ${propertyName}`);
    }
  });
  
  console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Setup ${subscriptions.length} observable property subscriptions`);
  return subscriptions;
}

// Combined SSE middleware for all properties
const combinedSSEMiddleware = createCombinedSSEMiddleware(PROPERTIES, sseManager);

// Read handlers for all properties
const readHandlers = createReadHandlers(['simStats', 'models', 'poses'], sseManager);
const readSimStats = readHandlers.simStats;
const readModels = readHandlers.models;
const readPoses = readHandlers.poses;

// Cleanup function (re-export from common)
const cleanupSubscriptions = cleanupSubs;

module.exports = {
  // Read handlers
  readSimStats,
  readModels,
  readPoses,
  
  // Middleware
  combinedSSEMiddleware,
  
  // Setup and cleanup
  setupAllObservableProperties,
  cleanupSubscriptions
};

