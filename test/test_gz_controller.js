const fs = require("fs");
const fsp = require("fs/promises");
const path = require("path");
const { spawn } = require("child_process");
const rclnodejs = require("rclnodejs");
const { Servient } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");
const baseDir = path.join(process.env.HOME, "wos");

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

    this.thing.setActionHandler("publishMessage", async (input) => {
      const msg = await input.value();
      this.publisher.publish({ data: msg });
      console.log(`[WoT Action] Published: ${msg}`);
      return `Published: ${msg}`;
    });

    async function resolveFilePath(fileName) {
      const searchPaths = [
        ["resource", "world"],
        ["resource", "model"],
        ["resource", "launch"],
        ["upload", "world"],
        ["upload", "model"],
        ["upload", "launch"]
      ];

      for (const [group, subdir] of searchPaths) {
        const fullPath = path.join(baseDir, group, subdir, fileName);
        try {
          const stat = await fs.promises.stat(fullPath);
          if (stat.isFile()) {
            return fullPath;
          }
        } catch (e) {
        }
      }
      console.warn("File NOT FOUND:", fileName);
      return null;
    }

    this.thing.setActionHandler("launchSimulation", async (input) => {
      const data = await input.value();
      const method = data?.method || "gz";
      const fileName = data?.fileName;
      const args = data?.arguments || [];

      if (!fileName) throw new Error("Missing fileName for launchSimulation.");

      const fullPath = await resolveFilePath(fileName);
      if (!fullPath) throw new Error(`File "${fileName}" not found in resource or upload folders`);

      if (this.gzProcess) throw new Error("Simulation already running.");

      try {
        const cmd = method === "gz" ? "gz" : "ros2";
        const cmdArgs = method === "gz"
          ? ["sim", fullPath, ...args]
          : ["launch", fullPath, ...args];

        console.log(`[launchSimulation] Spawning: ${cmd} ${cmdArgs.join(" ")}`);

        this.gzProcess = spawn(cmd, cmdArgs, {
          detached: true,
          stdio: "ignore"
        });
        this.simPid = this.gzProcess.pid;
        this.gzProcess.unref();

        return `Simulation launched with: ${fullPath}`;
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

    this.thing.setPropertyReadHandler("availableResources", async () => {

      async function listFiles(subdir) {
        try {
          const entries = await fsp.readdir(subdir, { withFileTypes: true });
          return entries.filter(e => e.isFile()).map(e => e.name);
        } catch (e) {
          console.warn(`Failed to read ${subdir}: ${e.message}`);
          return [];
        }
      }

      async function listAll(dirType) {
        const result = {};
        for (const type of ["model", "world", "launch"]) {
          const fullPath = path.join(baseDir, dirType, type);
          result[type] = await listFiles(fullPath);
        }
        return result;
      }

      return {
        resource: await listAll("resource"),
        upload: await listAll("upload")
      };
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
