const fs = require("fs");
const path = require("path");
const rclnodejs = require("rclnodejs");
const { Servient } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");

const { makeSimControl } = require("../library/coppeliasim/cs_action_handlers");

class CoppeliaSimController {
  constructor(tdPath = "./cs_controller.json", rosTopic = "wot_topic", port = 8080) {
    this.tdPath = tdPath;
    this.rosTopic = rosTopic;
    this.port = port;
    this.spinInterval = null;
    this.observableSubscriptions = [];
  }

  async init() {
    await rclnodejs.init();
    this.node = new rclnodejs.Node("coppeliasim_wot_node");
    this.publisher = this.node.createPublisher("std_msgs/msg/String", this.rosTopic);
    this.observableSubscriptions = [];
    this.startSpin();

    this.servient = new Servient();
    this.servient.addServer(new HttpServer({ port: this.port }));

    const td = JSON.parse(fs.readFileSync(this.tdPath, "utf8"));

    const wot = await this.servient.start();
    this.thing = await wot.produce(td);

    // Set up action handlers
    this.thing.setActionHandler("simControl", makeSimControl(this.node));

    // Set up property handlers (placeholder for now)
    this.thing.setPropertyReadHandler("simStats", this.readSimStats.bind(this));
    this.thing.setPropertyReadHandler("models", this.readModels.bind(this));
    this.thing.setPropertyReadHandler("poses", this.readPoses.bind(this));

    await this.thing.expose();
    console.log(`CoppeliaSim Thing exposed at http://localhost:${this.port}/cs_controller`);
  }

  startSpin() {
    this.spinInterval = setInterval(() => {
      rclnodejs.spinOnce(this.node);
    }, 100);
  }

  async stop() {
    if (this.spinInterval) {
      clearInterval(this.spinInterval);
    }
    
    // Cleanup observable property subscriptions
    this.observableSubscriptions.forEach(sub => {
      if (sub && typeof sub.destroy === 'function') {
        sub.destroy();
      }
    });
    this.observableSubscriptions = [];
    
    if (this.node) {
      this.node.destroy();
    }
    await rclnodejs.shutdown();

    console.log("Gracefully shut down CoppeliaSim Controller.");
  }

  /**
   * Read simulation statistics from CoppeliaSim
   * Placeholder implementation - would subscribe to /coppeliasim/stats topic
   */
  async readSimStats() {
    // TODO: Implement subscription to /coppeliasim/stats topic
    return {
      timestamp: new Date().toISOString(),
      simTime: { sec: 0, nsec: 0 },
      simState: 0,
      speedFactor: 0,
      realTime: { sec: 0, nsec: 0 }
    };
  }

  /**
   * Read available models from CoppeliaSim
   * Placeholder implementation - would subscribe to /coppeliasim/models topic
   */
  async readModels() {
    // TODO: Implement subscription to /coppeliasim/models topic
    return {
      timestamp: new Date().toISOString(),
      models: []
    };
  }

  /**
   * Read object poses from CoppeliaSim
   * Placeholder implementation - would subscribe to /coppeliasim/poses topic
   */
  async readPoses() {
    // TODO: Implement subscription to /coppeliasim/poses topic
    return {
      timestamp: new Date().toISOString(),
      poses: []
    };
  }
}

async function main() {
  const controller = new CoppeliaSimController();
  await controller.init();

  process.on("SIGINT", async () => {
    console.log("\nCaught SIGINT. Shutting down...");
    await controller.stop();
    process.exit(0);
  });
}

main().catch((err) => {
  console.error("Error in CoppeliaSim controller:", err);
});
