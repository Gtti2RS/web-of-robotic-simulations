const { Servient, Helpers } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");
const { spawn } = require('child_process');
const fs = require("fs");
const path = require('path');

const servient = new Servient();
servient.addServer(new HttpServer({ port: 8085 }));

const td = JSON.parse(fs.readFileSync("sim.json", "utf-8"));
const WORLD_DIR = path.join(__dirname, 'world');
let currentProcess = null;

servient.start().then(async (WoT) => {
  const thing = await WoT.produce(td);

 thing.setPropertyReadHandler("world_list", async () => {
      const files = fs.readdirSync(WORLD_DIR).filter(f =>
        f.endsWith(".sdf") || f.endsWith(".world") || f.endsWith(".xml")
      );
      return files;
    });

 thing.setActionHandler("launch", async (params) => {
      const worldFile = await params.value();
      const worldPath = path.join(WORLD_DIR, worldFile);
      if (!fs.existsSync(worldPath)) {
        throw new Error(`World file not found: ${worldFile}`);
      }
      // Stop previous sim if running
      if (currentProcess) currentProcess.kill();

      currentProcess = spawn("gz", ["sim", worldPath], {
        detached: true,
        stdio: 'ignore'
      });

      currentProcess.unref(); // Allow parent to exit independently
      return `Started simulation: ${worldFile}`;
    });

 thing.setActionHandler("run", async () => {
    console.log("Simulation started");
    return "Simulation started";
  });

 thing.setActionHandler("pause", async () => {
    console.log("Simulation paused");
    return "Simulation paused";
  });

 thing.setActionHandler("reset", async () => {
    console.log("Simulation reset");
    return "Simulation reset";
  });
 thing.setActionHandler("spawn", async (params) => {
  const input = await params.value(); // resolves WoT.Value
  const { name, type } = input;
  console.log("Spawning entity:", name, type);
  return `Spawned ${name} of type ${type}`;
});

thing.setActionHandler("exit", async () => {
      if (currentProcess) {
        currentProcess.kill();
        currentProcess = null;
        return "Simulation stopped";
      } else {
        return "No active simulation";
      }
    });

  await thing.expose();
  console.log(`TD available at: http://localhost:8080/${thing.getThingDescription().title}`);
});
