const rclnodejs = require("rclnodejs");

// Generic SSE manager for observable properties
class SSEManager {
  constructor() {
    this.clients = new Map(); // Map of property name -> Set of clients
    this.data = new Map(); // Map of property name -> current data
    this.keepAliveInterval = null;
    this.startKeepAlive();
  }

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

  registerProperty(propertyName, initialData = null) {
    if (!this.clients.has(propertyName)) {
      this.clients.set(propertyName, new Set());
      this.data.set(propertyName, initialData);
    }
  }

  addClient(propertyName, res) {
    if (!this.clients.has(propertyName)) {
      this.registerProperty(propertyName);
    }
    
    const clients = this.clients.get(propertyName);
    clients.add(res);
    
    const currentData = this.data.get(propertyName);
    if (currentData !== null) {
      res.write(`data: ${JSON.stringify(currentData)}\n\n`);
    }
    
    res.on("close", () => clients.delete(res));
  }

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

  getData(propertyName) {
    return this.data.get(propertyName);
  }

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
        return;
      }
      next();
    };
  }

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

// Property configurations
const PROPERTIES = {
  sim_stats: {
    topic: "/stats_json",
    messageType: "std_msgs/msg/String",
    initialData: { timestamp: new Date().toISOString(), message: "No stats received yet" },
    ssePath: "/gz_controller/properties/sim_stats/observable"
  },
  poses: {
    topic: "/pose/info_json", 
    messageType: "std_msgs/msg/String",
    initialData: { timestamp: new Date().toISOString(), message: "No poses received yet" },
    ssePath: "/gz_controller/properties/poses/observable"
  }
};

// Initialize all properties
Object.entries(PROPERTIES).forEach(([name, config]) => {
  sseManager.registerProperty(name, config.initialData);
});

// Generic ROS2 topic subscription creator
function createROS2TopicSubscription(node, topicName, messageType, propertyName, messageHandler) {
  if (!node) {
    console.warn("ROS2 node not available for subscription");
    return null;
  }

  try {
    const subscriber = node.createSubscription(messageType, topicName, (msg) => {
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
    });
    
    console.log(`Subscribed to ROS2 topic ${topicName} for ${propertyName}`);
    return subscriber;
  } catch (error) {
    console.error(`Error setting up subscription for ${topicName}:`, error);
    return null;
  }
}

// JSON message handler for string topics
const jsonMessageHandler = (msg) => {
  try {
    return JSON.parse(msg.data);
  } catch (error) {
    throw new Error(`Failed to parse JSON: ${error.message}`);
  }
};

// Setup all observable properties with ROS2 subscriptions
function setupAllObservableProperties(node) {
  const subscriptions = [];
  
  Object.entries(PROPERTIES).forEach(([propertyName, config]) => {
    const subscriber = createROS2TopicSubscription(
      node, 
      config.topic, 
      config.messageType, 
      propertyName, 
      jsonMessageHandler
    );
    if (subscriber) subscriptions.push(subscriber);
  });
  
  console.log(`Setup ${subscriptions.length} observable property subscriptions`);
  return subscriptions;
}

// Combined SSE middleware for all properties
const combinedSSEMiddleware = async (req, res, next) => {
  for (const [propertyName, config] of Object.entries(PROPERTIES)) {
    if (req.method === "GET" && req.url.split("?")[0] === config.ssePath) {
      return sseManager.createSSEMiddleware(propertyName, config.ssePath)(req, res, next);
    }
  }
  next();
};

// Read handlers for all properties
const readHandlers = {};
Object.keys(PROPERTIES).forEach(propertyName => {
  readHandlers[propertyName] = async () => sseManager.getData(propertyName);
});

// Cleanup function
function cleanupSubscriptions(subscriptions) {
  if (subscriptions && Array.isArray(subscriptions)) {
    subscriptions.forEach(sub => {
      if (sub && typeof sub.destroy === 'function') {
        sub.destroy();
      }
    });
  }
}

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
  
  // Middleware
  combinedSSEMiddleware,
  
  // Setup and cleanup
  setupAllObservableProperties,
  cleanupSubscriptions
};
