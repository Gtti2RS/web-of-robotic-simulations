const fs = require("fs");
const path = require("path");
const { spawn } = require("child_process");
const rclnodejs = require("rclnodejs");
const { Servient } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");

class WotPublisherServer {
  constructor(tdPath = "./gz_controller.json", rosTopic = "wot_topic", port = 8080) {
    this.tdPath = tdPath;
    this.rosTopic = rosTopic;
    this.port = port;
    this.spinInterval = null;
    this.gzProcess = null;
    this.simPid = null;
    this.uploadDirs = {
      world: path.join(process.env.HOME, "wos", "world"),
      model: path.join(process.env.HOME, "wos", "model"),
      launch: path.join(process.env.HOME, "wos", "launch")
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

    this.thing.setActionHandler("publishMessage", async (input) => {
      const msg = await input.value();
      this.publisher.publish({ data: msg });
      console.log(`[WoT Action] Published: ${msg}`);
      return `Published: ${msg}`;
    });

    this.thing.setActionHandler("launchSimulation", async (input) => {
      const data = await input.value();
      const method = data?.method || "gz";
      const fileName = data?.fileName;
      const args = data?.arguments || [];

      if (!fileName) throw new Error("Missing fileName for launchSimulation.");

      const targetDir = method === "ros2" ? this.uploadDirs.launch : this.uploadDirs.world;
      const filePath = path.join(targetDir, fileName);
      const expandedPath = filePath.replace(/^~\//, `${process.env.HOME}/`);

      if (this.gzProcess) throw new Error("Simulation already running.");

      try {
        const cmd = method === "gz" ? "gz" : "ros2";
        const cmdArgs = method === "gz" ? ["sim", expandedPath, ...args] : ["launch", expandedPath, ...args];
        console.log(`[launchSimulation] Spawning: ${cmd} ${cmdArgs.join(" ")}`);

        this.gzProcess = spawn(cmd, cmdArgs, {
          detached: true,
          stdio: "ignore"
        });
        this.simPid = this.gzProcess.pid;
        this.gzProcess.unref();
        return `Simulation launched with: ${expandedPath}`;
      } catch (error) {
        console.error(`[${method.toUpperCase()} Error]`, error.message);
        this.gzProcess = null;
        this.simPid = null;
        throw new Error(`Failed to launch simulation: ${error.message}`);
      }
    });

    this.thing.setActionHandler("exitSimulation", async () => {
      if (this.gzProcess && this.simPid) {
        try {
          process.kill(-this.simPid, "SIGINT");
          console.log("[WoT Action] Sent SIGINT to simulation group:", -this.simPid);
        } catch (e) {
          console.warn("[WoT Action] Error killing process group:", e.message);
        }
        this.gzProcess = null;
        this.simPid = null;
        return "Simulation exited.";
      } else {
        return "No simulation is currently running.";
      }
    });

    this.thing.setActionHandler("uploadFile", async (input) => {
      const data = await input.value();
      const { name, content, target } = data;
      if (!name || !content || !target || !(target in this.uploadDirs)) {
        throw new Error("Invalid input: name, content, and valid target ('world', 'model', or 'launch') required.");
      }
      const filePath = path.join(this.uploadDirs[target], name);
      fs.writeFileSync(filePath, content);
      console.log(`[WoT Action] Uploaded ${target} file: ${filePath}`);
      return `Uploaded to ${target}: ${filePath}`;
    });

    this.thing.setPropertyReadHandler("uploadedWorlds", async () => {
      return fs.readdirSync(this.uploadDirs.world);
    });

    this.thing.setPropertyReadHandler("uploadedModels", async () => {
      return fs.readdirSync(this.uploadDirs.model);
    });

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
