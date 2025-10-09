const fs = require("fs");
const path = require("path");
const rclnodejs = require("rclnodejs");
const { Servient } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");

const { makeSimControl, makeManageScene, makeManageModel, stopAllUR10Processes } = require("../library/coppeliasim/cs_action_handlers");
const { readCoppeliaSimAssets } = require("../library/common/fileUtils");
const { makePublishMessage, makeSendRos2Cmd } = require("../library/common/ros2_utils");
const { readSimStats, readModels, readPoses, combinedSSEMiddleware, setupAllObservableProperties, cleanupSubscriptions } = require("../library/coppeliasim/cs_observable_topics");

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
    this.servient.addServer(new HttpServer({ port: this.port, middleware: combinedSSEMiddleware }));

    const td = JSON.parse(fs.readFileSync(this.tdPath, "utf8"));

    const wot = await this.servient.start();
    this.thing = await wot.produce(td);

    // Set up action handlers
    this.thing.setActionHandler("simControl", makeSimControl(this.node));
    this.thing.setActionHandler("manageScene", makeManageScene(this.node));
    this.thing.setActionHandler("manageModel", makeManageModel(this.node));
    this.thing.setActionHandler("publishMessage", makePublishMessage(this.node));
    this.thing.setActionHandler("sendRos2Cmd", makeSendRos2Cmd(this.node));

    // Set up property handlers
    this.thing.setPropertyReadHandler("visualize", () => true); // CoppeliaSim always has visualization
    this.thing.setPropertyReadHandler("assets", readCoppeliaSimAssets);
    this.thing.setPropertyReadHandler("simStats", readSimStats);
    this.thing.setPropertyReadHandler("models", readModels);
    this.thing.setPropertyReadHandler("poses", readPoses);

    // Setup observable property subscriptions
    const subscriptions = await setupAllObservableProperties(this.node);
    this.observableSubscriptions = subscriptions;
    console.log(`Set up ${subscriptions.length} observable property subscriptions`);

    await this.thing.expose();
    console.log(`CoppeliaSim Thing exposed at http://localhost:${this.port}/cs_controller`);

    // Automatically start simulation
    console.log("Auto-starting simulation...");
    try {
      const simControlHandler = makeSimControl(this.node);
      const result = await simControlHandler({
        value: async () => ({ mode: 'run' })
      });
      if (result.success) {
        console.log("✓ Simulation started automatically:", result.message);
      } else {
        console.warn("⚠ Failed to auto-start simulation:", result.message);
      }
    } catch (error) {
      console.error("✗ Error auto-starting simulation:", error.message);
    }
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
    
    // Stop all UR10 child processes
    console.log("Stopping UR10 child processes...");
    stopAllUR10Processes();
    
    // Cleanup observable property subscriptions
    cleanupSubscriptions(this.observableSubscriptions);
    this.observableSubscriptions = [];
    
    if (this.node) {
      this.node.destroy();
    }
    await rclnodejs.shutdown();

    console.log("Gracefully shut down CoppeliaSim Controller.");
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
