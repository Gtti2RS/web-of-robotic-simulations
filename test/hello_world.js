const fs = require("fs");
const rclnodejs = require("rclnodejs");
const { Servient } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");

class WotPublisherServer {
  constructor(tdPath = "./td.json", rosTopic = "wot_topic", port = 8080) {
    this.tdPath = tdPath;
    this.rosTopic = rosTopic;
    this.port = port;
    this.spinInterval = null;
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

    this.thing.setActionHandler("publishMessage", async (input) => {
      const data = await input.value();
      const msg = data.toString();
      this.publisher.publish({ data: msg });
      console.log(`[WoT Action] Published: ${msg}`);
      return `Published: ${msg}`;
    });

    await this.thing.expose();
    console.log(`Thing exposed at http://localhost:${this.port}/ros2publisher`);
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
