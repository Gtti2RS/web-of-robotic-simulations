/**
 * @fileoverview SSE Manager for Observable Properties
 * 
 * This module provides a generic Server-Sent Events (SSE) manager for Web of Things
 * observable properties, enabling real-time data streaming to web clients.
 * 
 * Key Features:
 * - Multi-property SSE management
 * - Automatic keep-alive for maintaining connections
 * - Client lifecycle management (connect/disconnect)
 * - Real-time data broadcasting to all connected clients
 * - Middleware creation for HTTP servers
 * 
 * Usage:
 * - Register properties with initial data
 * - Update data to broadcast to all clients
 * - Create SSE middleware for HTTP endpoints
 * 
 * @author Yifan & Cursor
 * @version 1.0.0
 */

/**
 * SSE Manager class for managing Server-Sent Events connections
 * and real-time data streaming for observable properties
 */
class SSEManager {
  constructor() {
    this.clients = new Map(); // Map of property name -> Set of clients
    this.data = new Map(); // Map of property name -> current data
    this.keepAliveInterval = null;
    this.startKeepAlive();
  }

  /**
   * Start keep-alive interval to maintain SSE connections
   */
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

  /**
   * Register a new property for SSE streaming
   * @param {string} propertyName - Name of the property
   * @param {*} initialData - Initial data for the property
   */
  registerProperty(propertyName, initialData = null) {
    if (!this.clients.has(propertyName)) {
      this.clients.set(propertyName, new Set());
      this.data.set(propertyName, initialData);
    }
  }

  /**
   * Add a client (HTTP response object) to receive updates for a property
   * @param {string} propertyName - Name of the property
   * @param {Object} res - HTTP response object
   */
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

  /**
   * Update data for a property and broadcast to all connected clients
   * @param {string} propertyName - Name of the property
   * @param {*} data - New data to broadcast
   */
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

  /**
   * Get current data for a property
   * @param {string} propertyName - Name of the property
   * @returns {*} Current data
   */
  getData(propertyName) {
    return this.data.get(propertyName);
  }

  /**
   * Create SSE middleware for a property
   * @param {string} propertyName - Name of the property
   * @param {string} ssePath - SSE endpoint path
   * @returns {Function} Express/Connect middleware function
   */
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

  /**
   * Cleanup all resources
   */
  cleanup() {
    if (this.keepAliveInterval) {
      clearInterval(this.keepAliveInterval);
      this.keepAliveInterval = null;
    }
    this.clients.clear();
    this.data.clear();
  }
}

module.exports = { SSEManager };

