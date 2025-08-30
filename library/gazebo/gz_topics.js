const rclnodejs = require("rclnodejs");

// Generic SSE manager for observable properties
class SSEManager {
  constructor() {
    this.clients = new Map(); // Map of property name -> Set of clients
    this.data = new Map(); // Map of property name -> current data
    this.keepAliveInterval = null;
    this.startKeepAlive();
  }

  // Start keep-alive pings for all SSE connections
  startKeepAlive() {
    if (this.keepAliveInterval) return;
    
    this.keepAliveInterval = setInterval(() => {
      for (const [propertyName, clients] of this.clients) {
        for (const res of clients) {
          res.write(": keep-alive\n\n");
        }
      }
    }, 15000);
  }

  // Register a new observable property
  registerProperty(propertyName, initialData = null) {
    if (!this.clients.has(propertyName)) {
      this.clients.set(propertyName, new Set());
      this.data.set(propertyName, initialData);
    }
  }

  // Add a client to a property's SSE stream
  addClient(propertyName, res) {
    if (!this.clients.has(propertyName)) {
      this.registerProperty(propertyName);
    }
    
    const clients = this.clients.get(propertyName);
    clients.add(res);
    
    // Send current data immediately
    const currentData = this.data.get(propertyName);
    if (currentData !== null) {
      res.write(`data: ${JSON.stringify(currentData)}\n\n`);
    }
    
    // Clean up when client disconnects
    res.on("close", () => {
      clients.delete(res);
    });
  }

  // Update data for a property and broadcast to all clients
  updateData(propertyName, data) {
    this.data.set(propertyName, data);
    const clients = this.clients.get(propertyName);
    if (clients) {
      const line = `data: ${JSON.stringify(data)}\n\n`;
      for (const res of clients) {
        res.write(line);
      }
    }
  }

  // Get current data for a property
  getData(propertyName) {
    return this.data.get(propertyName);
  }

  // Create SSE middleware for a specific property
  createSSEMiddleware(propertyName, ssePath) {
    return async (req, res, next) => {
      if (req.method === "GET" && req.url.split("?")[0] === ssePath) {
        res.writeHead(200, {
          "Content-Type": "text/event-stream",
          "Cache-Control": "no-cache",
          "Connection": "keep-alive",
          "Access-Control-Allow-Origin": "*"
        });
        
        this.addClient(propertyName, res);
        return; // Don't pass to Node-WoT routing
      }
      next();
    };
  }

  // Cleanup
  cleanup() {
    if (this.keepAliveInterval) {
      clearInterval(this.keepAliveInterval);
      this.keepAliveInterval = null;
    }
    this.clients.clear();
    this.data.clear();
  }
}

// Global SSE manager instance
const sseManager = new SSEManager();

// Initialize sim_stats property
sseManager.registerProperty("sim_stats", {
  timestamp: new Date().toISOString(),
  message: "No stats received yet"
});

// Read handler for sim_stats property
async function readSimStats() {
  return sseManager.getData("sim_stats");
}

// SSE middleware for sim_stats observable property
const simStatsSSEMiddleware = sseManager.createSSEMiddleware(
  "sim_stats", 
  "/gz_controller/properties/sim_stats/observable"
);

// Function to update stats from ROS2 topic
function updateStatsFromROS2(statsData) {
  const updatedStats = {
    ...statsData,
    timestamp: new Date().toISOString()
  };
  sseManager.updateData("sim_stats", updatedStats);
}

// Generic function to create ROS2 topic subscription for observable properties
function createROS2TopicSubscription(node, topicName, messageType, propertyName, messageHandler) {
  if (!node) {
    console.warn("ROS2 node not available for subscription");
    return null;
  }

  try {
    const subscriber = node.createSubscription(
      messageType,
      topicName,
      (msg) => {
        try {
          const processedData = messageHandler ? messageHandler(msg) : msg;
          sseManager.updateData(propertyName, {
            ...processedData,
            timestamp: new Date().toISOString()
          });
        } catch (error) {
          console.error(`Error processing message from ${topicName}:`, error);
          sseManager.updateData(propertyName, {
            error: `Failed to process message from ${topicName}`,
            raw_data: msg,
            timestamp: new Date().toISOString()
          });
        }
      }
    );
    
    console.log(`Subscribed to ROS2 topic ${topicName} for ${propertyName}`);
    return subscriber;
  } catch (error) {
    console.error(`Error setting up subscription for ${topicName}:`, error);
    return null;
  }
}

// Function to setup ROS2 subscription for /stats_json topic
function setupStatsSubscription(node) {
  return createROS2TopicSubscription(
    node,
    "/stats_json",
    "std_msgs/msg/String",
    "sim_stats",
    (msg) => {
      try {
        return JSON.parse(msg.data);
      } catch (error) {
        throw new Error(`Failed to parse JSON: ${error.message}`);
      }
    }
  );
}

// Generic function to create a new observable property with SSE support
function createObservableProperty(propertyName, initialData = null, ssePath = null) {
  sseManager.registerProperty(propertyName, initialData);
  
  return {
    readHandler: async () => sseManager.getData(propertyName),
    sseMiddleware: ssePath ? sseManager.createSSEMiddleware(propertyName, ssePath) : null,
    updateData: (data) => sseManager.updateData(propertyName, data),
    getData: () => sseManager.getData(propertyName)
  };
}

// Generic function to create a complete observable property with ROS2 subscription
function createROS2ObservableProperty(node, propertyName, topicName, messageType, messageHandler, initialData = null, ssePath = null) {
  // Create the observable property
  const property = createObservableProperty(propertyName, initialData, ssePath);
  
  // Setup ROS2 subscription
  const subscriber = createROS2TopicSubscription(node, topicName, messageType, propertyName, messageHandler);
  
  return {
    ...property,
    subscriber,
    topicName,
    messageType
  };
}

// Function to setup all observable properties with their ROS2 subscriptions
function setupAllObservableProperties(node) {
  const subscriptions = [];
  
  // Setup sim_stats property
  const simStatsSub = setupStatsSubscription(node);
  if (simStatsSub) subscriptions.push(simStatsSub);
  
  // Example: Add more observable properties here
  // const temperatureProperty = createROS2ObservableProperty(
  //   node,
  //   "temperature",
  //   "/temperature",
  //   "std_msgs/msg/Float32",
  //   (msg) => ({ value: msg.data, unit: "celsius" }),
  //   { value: 20, unit: "celsius" },
  //   "/gz_controller/properties/temperature/observable"
  // );
  // subscriptions.push(temperatureProperty.subscriber);
  
  console.log(`Setup ${subscriptions.length} observable property subscriptions`);
  return subscriptions;
}

// Function to cleanup all subscriptions
function cleanupSubscriptions(subscriptions) {
  if (subscriptions && Array.isArray(subscriptions)) {
    subscriptions.forEach(sub => {
      if (sub && typeof sub.destroy === 'function') {
        sub.destroy();
      }
    });
  }
}

module.exports = {
  readSimStats,
  simStatsSSEMiddleware,
  setupStatsSubscription,
  createObservableProperty,
  createROS2ObservableProperty,
  createROS2TopicSubscription,
  setupAllObservableProperties,
  cleanupSubscriptions,
  sseManager
};
