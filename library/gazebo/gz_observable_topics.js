/**
 * @fileoverview Gazebo Observable Topics Library
 * 
 * This module provides real-time data streaming from Gazebo simulation via ROS 2 topics,
 * implementing Server-Sent Events (SSE) for Web of Things observable properties.
 * 
 * Key Features:
 * - Real-time simulation statistics streaming (simStats)
 * - Live entity pose monitoring (poses)
 * - Dynamic model list tracking (models)
 * - Server-Sent Events (SSE) implementation for web clients
 * - Automatic topic subscription management
 * - Connection lifecycle management with keep-alive
 * 
 * Observable Properties:
 * - simStats: Simulation timing, iterations, real-time factor
 * - poses: Entity positions and orientations
 * - models: Available models in the simulation
 * 
 * @author Yifan & Cursor & ChatGPT
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



// Property configurations - topics will be dynamically updated with world name
const PROPERTIES = {
  sim_stats: {
    topic: "/stats_json", // Will be updated to /world/${world}/stats_json
    messageType: "std_msgs/msg/String",
    initialData: { timestamp: new Date().toISOString(), message: "No stats received yet" },
    ssePath: "/gz_controller/properties/sim_stats/observable"
  },
  poses: {
    topic: "/pose/info_json", // Will be updated to /world/${world}/pose/info_json
    messageType: "std_msgs/msg/String",
    initialData: { timestamp: new Date().toISOString(), message: "No poses received yet" },
    ssePath: "/gz_controller/properties/poses/observable"
  },
  models: {
    topic: "/pose/info_json", // Will be updated to /world/${world}/pose/info_json
    messageType: "std_msgs/msg/String", 
    initialData: { timestamp: new Date().toISOString(), message: "No models received yet" },
    ssePath: "/gz_controller/properties/models/observable"
  }
};

// Shared pose data for poses and models properties
let sharedPoseData = null;

// Initialize all properties
Object.entries(PROPERTIES).forEach(([name, config]) => {
  sseManager.registerProperty(name, config.initialData);
});


// Shared pose message handler - processes pose data for both poses and models properties
const sharedPoseMessageHandler = (msg) => {
  try {
    const data = JSON.parse(msg.data);
    sharedPoseData = data;
    
    // Update poses property with full pose data
    sseManager.updateData("poses", {
      ...data,
      timestamp: new Date().toISOString()
    });
    
    // Extract and update models property
    const poses = data.pose || [];
    
    // Find the first "link" entity (this marks the boundary between models and links)
    let linkIndex = -1;
    for (let i = 0; i < poses.length; i++) {
      if (poses[i].name === "link") {
        linkIndex = i;
        break;
      }
    }
    
    // Extract models (entities before the first "link")
    const models = [];
    const endIndex = linkIndex > 0 ? linkIndex : poses.length;
    
    // System entities to exclude
    const systemEntities = new Set([
      "ground_plane", "sun", "link", "visual", "collision", 
      "sunVisual", "base", "shoulder", "upperarm", "forearm", 
      "wrist1", "wrist2", "wrist3"
    ]);
    
    for (let i = 0; i < endIndex; i++) {
      const entity = poses[i];
      const name = entity.name;
      
      // Skip system entities
      if (name && !systemEntities.has(name)) {
        // Include all non-system entities, including those with underscores (like box_0, cylinder_0)
        models.push({
          name: name,
          id: entity.id
        });
      }
    }
    
    // Update models property
    sseManager.updateData("models", {
      models: models,
      total_count: models.length,
      timestamp: new Date().toISOString()
    });
    
    // Don't return data to avoid double processing
    return null;
  } catch (error) {
    throw new Error(`Failed to process pose data: ${error.message}`);
  }
};

// Setup all observable properties with ROS2 subscriptions
async function setupAllObservableProperties(node) {
  const subscriptions = [];
  const processedTopics = new Set();
  
  // Update topics with world name if available
  let world = null;
  try {
    const { get_world } = require('./gz_world_utils');
    world = get_world();
    
    if (world) {
      // Update topics to include world name
      PROPERTIES.sim_stats.topic = `/world/${world}/stats_json`;
      PROPERTIES.poses.topic = `/world/${world}/pose/info_json`;
      PROPERTIES.models.topic = `/world/${world}/pose/info_json`;
      
      console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Updated topics with world name: ${world}`);
    } else {
      console.warn(`[${new Date().toISOString()}] [setupAllObservableProperties] No active world detected, skipping topic subscriptions`);
      return []; // Return empty array - no subscriptions to create
    }
  } catch (error) {
    console.warn(`[${new Date().toISOString()}] [setupAllObservableProperties] No world name available, skipping topic subscriptions: ${error.message}`);
    return []; // Return empty array - no subscriptions to create
  }
  
  Object.entries(PROPERTIES).forEach(([propertyName, config]) => {
    console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Processing property: ${propertyName}, topic: ${config.topic}`);
    
    // Skip if we already processed this topic (for shared subscriptions)
    if (processedTopics.has(config.topic)) {
      console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Skipping ${propertyName} - topic ${config.topic} already processed`);
      return;
    }
    
    let messageHandler;
    if (config.topic.includes("/pose/info_json")) {
      // Use shared handler for pose data
      messageHandler = sharedPoseMessageHandler;
      processedTopics.add(config.topic);
      console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Using shared pose handler for ${propertyName}`);
    } else {
      // Use default JSON handler for other topics
      messageHandler = jsonMessageHandler;
      console.log(`[${new Date().toISOString()}] [setupAllObservableProperties] Using JSON handler for ${propertyName}`);
    }
    
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
const readHandlers = createReadHandlers(['sim_stats', 'poses', 'models'], sseManager);

// Cleanup function (re-export from common)
const cleanupSubscriptions = cleanupSubs;

// Generic property creator for future use
function createObservableProperty(propertyName, initialData = null, ssePath = null) {
  sseManager.registerProperty(propertyName, initialData);
  return {
    readHandler: async () => sseManager.getData(propertyName),
    sseMiddleware: ssePath ? sseManager.createSSEMiddleware(propertyName, ssePath) : null,
    updateData: (data) => sseManager.updateData(propertyName, data),
    getData: () => sseManager.getData(propertyName)
  };
}

function createROS2ObservableProperty(node, propertyName, topicName, messageType, messageHandler, initialData = null, ssePath = null) {
  const property = createObservableProperty(propertyName, initialData, ssePath);
  const subscriber = createROS2TopicSubscription(node, topicName, messageType, propertyName, messageHandler);
  return { ...property, subscriber, topicName, messageType };
}

module.exports = {
  // Read handlers
  readSimStats: readHandlers.sim_stats,
  readPoses: readHandlers.poses,
  readModels: readHandlers.models,
  
  // Middleware
  combinedSSEMiddleware,
  
  // Setup and cleanup
  setupAllObservableProperties,
  cleanupSubscriptions
};
