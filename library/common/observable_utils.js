/**
 * @fileoverview Observable Properties Utilities
 * 
 * This module provides common utilities for implementing WoT observable properties
 * with ROS 2 topics and Server-Sent Events (SSE).
 * 
 * Key Features:
 * - ROS 2 topic subscription creation
 * - Message handler utilities
 * - Subscription cleanup
 * - Generic observable property setup
 * 
 * @author Yifan & Cursor
 * @version 1.0.0
 */

/**
 * Create a ROS 2 topic subscription with SSE integration
 * @param {Object} node - ROS 2 node instance
 * @param {string} topicName - ROS 2 topic name
 * @param {string} messageType - ROS 2 message type
 * @param {string} propertyName - WoT property name
 * @param {Function} messageHandler - Handler to process messages
 * @param {Object} sseManager - SSE manager instance to update
 * @returns {Object|null} Subscription object or null if failed
 */
function createROS2TopicSubscription(node, topicName, messageType, propertyName, messageHandler, sseManager) {
  if (!node) {
    console.warn("ROS2 node not available for subscription");
    return null;
  }

  try {
    const subscriber = node.createSubscription(messageType, topicName, (msg) => {
      try {
        const processedData = messageHandler ? messageHandler(msg) : msg;
        
        if (processedData !== null) {
          sseManager.updateData(propertyName, {
            ...processedData,
            timestamp: new Date().toISOString()
          });
        }
      } catch (error) {
        console.error(`Error processing message from ${topicName}:`, error);
        sseManager.updateData(propertyName, {
          error: `Failed to process message from ${topicName}`,
          raw_data: msg,
          timestamp: new Date().toISOString()
        });
      }
    });
    
    console.log(`[${new Date().toISOString()}] [createROS2TopicSubscription] Subscribed to ${topicName} for ${propertyName}`);
    return subscriber;
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [createROS2TopicSubscription] Error setting up subscription for ${topicName}:`, error);
    return null;
  }
}

/**
 * Generic JSON message handler for string topics
 * Parses msg.data as JSON
 * @param {Object} msg - ROS 2 message
 * @returns {Object} Parsed JSON data
 */
const jsonMessageHandler = (msg) => {
  try {
    return JSON.parse(msg.data);
  } catch (error) {
    throw new Error(`Failed to parse JSON: ${error.message}`);
  }
};

/**
 * Cleanup ROS 2 subscriptions
 * @param {Array} subscriptions - Array of subscription objects
 */
function cleanupSubscriptions(subscriptions) {
  if (subscriptions && Array.isArray(subscriptions)) {
    subscriptions.forEach(sub => {
      if (sub && typeof sub.destroy === 'function') {
        sub.destroy();
      }
    });
  }
}

/**
 * Create combined SSE middleware for multiple properties
 * @param {Object} properties - Property configurations
 * @param {Object} sseManager - SSE manager instance
 * @returns {Function} Express/Connect middleware
 */
function createCombinedSSEMiddleware(properties, sseManager) {
  return async (req, res, next) => {
    for (const [propertyName, config] of Object.entries(properties)) {
      if (req.method === "GET" && req.url.split("?")[0] === config.ssePath) {
        return sseManager.createSSEMiddleware(propertyName, config.ssePath)(req, res, next);
      }
    }
    next();
  };
}

/**
 * Create read handlers for multiple properties
 * @param {Array} propertyNames - Array of property names
 * @param {Object} sseManager - SSE manager instance
 * @returns {Object} Object with read handlers
 */
function createReadHandlers(propertyNames, sseManager) {
  const handlers = {};
  propertyNames.forEach(propertyName => {
    handlers[propertyName] = async () => sseManager.getData(propertyName);
  });
  return handlers;
}

module.exports = {
  createROS2TopicSubscription,
  jsonMessageHandler,
  cleanupSubscriptions,
  createCombinedSSEMiddleware,
  createReadHandlers
};

