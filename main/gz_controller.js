const fs = require("fs");
const path = require("path");
const rclnodejs = require("rclnodejs");
const { Servient } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");

const { readGazeboAssets } = require("../library/common/fileUtils");
const { makePublishMessage, makeSendRos2Cmd } = require("../library/common/ros2_utils");
const {makeSimControl, makeManageScene, makeManageModel, visualizeRead, makeVisualizeWrite} = require("../library/gazebo/gz_ros2_srv");
const { readSimStats, readPoses, readModels, combinedSSEMiddleware, cleanupSubscriptions } = require("../library/gazebo/gz_observable_topics");

class WotPublisherServer {
  constructor(tdPath = "./gz_controller.json", rosTopic = "wot_topic", port = 8080) {
    this.tdPath = tdPath;
    this.rosTopic = rosTopic;
    this.port = port;
    this.spinInterval = null;
    this.observableSubscriptions = [];
    this.uploadDirs = {
      world: path.join(process.env.HOME, "wos/upload", "world"),
      model: path.join(process.env.HOME, "wos/upload", "model"),
      launch: path.join(process.env.HOME, "wos/upload", "launch")
    };
    Object.values(this.uploadDirs).forEach(dir => fs.mkdirSync(dir, { recursive: true }));
  }

  async init() {
    await rclnodejs.init();
    this.node = new rclnodejs.Node("wot_pub_node");
    //setRosNode(this.node);
    this.publisher = this.node.createPublisher("std_msgs/msg/String", this.rosTopic);
    this.observableSubscriptions = []; // Will be set up when simulation is launched
    this.startSpin();

    this.servient = new Servient();
    this.servient.addServer(new HttpServer({ port: this.port, middleware: combinedSSEMiddleware }));

    const td = JSON.parse(fs.readFileSync(this.tdPath, "utf8"));

    const wot = await this.servient.start();
    this.thing = await wot.produce(td);

    this.thing.setPropertyReadHandler("assets", readGazeboAssets);
    this.thing.setPropertyReadHandler('visualize', visualizeRead);
    this.thing.setPropertyWriteHandler('visualize', makeVisualizeWrite(this.node));
    this.thing.setPropertyReadHandler('simStats', readSimStats);
    this.thing.setPropertyReadHandler('poses', readPoses);
    this.thing.setPropertyReadHandler('models', readModels);
    this.thing.setActionHandler("publishMessage", makePublishMessage(this.node));
    this.thing.setActionHandler("sendRos2Cmd", makeSendRos2Cmd(this.node));
    this.thing.setActionHandler('simControl', makeSimControl(this.node));
    this.thing.setActionHandler('manageScene', makeManageScene(this.node));
    this.thing.setActionHandler('manageModel', makeManageModel(this.node));

    await this.thing.expose();
    console.log(`Thing exposed at http://localhost:${this.port}/gz_controller`);
    
    // Initialize with empty.sdf as the default scene using manageScene
    try {
      console.log('Initializing with empty.sdf as default scene...');
      const manageScene = makeManageScene(this.node);
      const result = await manageScene({ 
        value: async () => ({ 
          mode: 'load',
          fileName: 'empty.sdf'
        }) 
      });
      if (result.success) {
        console.log('Default scene (empty.sdf) loaded successfully');
      } else {
        console.warn('Warning: Failed to load default scene:', result.message);
      }
    } catch (error) {
      console.warn('Warning: Failed to load default scene (empty.sdf):', error.message);
      console.warn('You can load a scene manually using the manageScene action');
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
    
    // Cleanup observable property subscriptions
    cleanupSubscriptions(this.observableSubscriptions);
    this.observableSubscriptions = [];
    
    if (this.node) {
      this.node.destroy();
    }
    await rclnodejs.shutdown();

    console.log("Gracefully shut down WoT Publisher Server.");
  }
}

async function main() {
  const server = new WotPublisherServer();
  await server.init();

  process.on("SIGINT", async () => {
    console.log("\nCaught SIGINT. Shutting down...");
    await server.stop();
    process.exit(0);
  });
}

main().catch((err) => {
  console.error("Error in WoT publisher server:", err);
});
