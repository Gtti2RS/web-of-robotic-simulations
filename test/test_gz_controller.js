const fs = require("fs");
const path = require("path");
const rclnodejs = require("rclnodejs");
const { Servient } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");

const { handleUploadFile, readAvailableResources } = require("../library/common/fileUtils");
const { launchSimulation, exitSimulation } = require("../library/gazebo/gz_actions");
const { publishMessage, sendRos2Cmd } = require("../library/common/ros2_utils");

class WotPublisherServer {
  constructor(tdPath = "./gz_controller.json", rosTopic = "wot_topic", port = 8080) {
    this.tdPath = tdPath;
    this.rosTopic = rosTopic;
    this.port = port;
    this.spinInterval = null;
    this.gzProcess = null;
    this.simPid = null;
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
    this.publisher = this.node.createPublisher("std_msgs/msg/String", this.rosTopic);
    this.startSpin();

    this.servient = new Servient();
    this.servient.addServer(new HttpServer({ port: this.port }));

    const td = JSON.parse(fs.readFileSync(this.tdPath, "utf8"));

    const wot = await this.servient.start();
    this.thing = await wot.produce(td);

    // this.thing.setActionHandler("publishMessage", async (input) => {
    //   const msg = await input.value();
    //   this.publisher.publish({ data: msg });
    //   console.log(`[WoT Action] Published: ${msg}`);
    //   return `Published: ${msg}`;
    // });
    this.thing.setActionHandler("publishMessage", (input) =>
      publishMessage(input, this.node)
    );
    this.thing.setActionHandler("launchSimulation", launchSimulation.bind(this));
    this.thing.setActionHandler("exitSimulation", exitSimulation.bind(this));
    this.thing.setActionHandler("uploadFile", handleUploadFile.bind(this));
    this.thing.setPropertyReadHandler("availableResources", async () => {
      return await readAvailableResources();
    });
    this.thing.setActionHandler("send_ros2_cmd", sendRos2Cmd.bind(this));
    await this.thing.expose();
    console.log(`Thing exposed at http://localhost:${this.port}/`);
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
    if (this.node) {
      this.node.destroy();
    }
    await rclnodejs.shutdown();

    if (this.gzProcess && this.simPid) {
      try {
        process.kill(-this.simPid, "SIGINT");
        console.log("[Shutdown] Sent SIGINT to simulation group:", -this.simPid);
      } catch (e) {
        console.warn("[Shutdown] Error killing process group:", e.message);
      }
      this.gzProcess = null;
      this.simPid = null;
    }

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
