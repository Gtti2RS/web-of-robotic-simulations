const { spawn } = require("child_process");
const { resolveFilePath } = require("../common/fileUtils");

async function launchSimulation(input) {
  const data = await input.value();
  const method = data?.method || "gz";
  const fileName = data?.fileName;
  const args = data?.arguments || [];

  if (!fileName) throw new Error("Missing fileName for launchSimulation");

  const fullPath = await resolveFilePath(fileName);
  if (!fullPath) throw new Error(`File "${fileName}" not found in resource or upload folders`);

  if (this.gzProcess) throw new Error("Simulation already running");

  const cmd = method === "gz" ? "gz" : "ros2";
  const cmdArgs = method === "gz"
    ? ["sim", fullPath, ...args]
    : ["launch", fullPath, ...args];

  console.log(`[launchSimulation] Spawning: ${cmd} ${cmdArgs.join(" ")}`);

  try {
    this.gzProcess = spawn(cmd, cmdArgs, {
      detached: true,
      stdio: "ignore"
    });
    this.simPid = this.gzProcess.pid;
    this.gzProcess.unref();
    return `Simulation launched with: ${fullPath}`;
  } catch (err) {
    console.error(`[${cmd} error]`, err.message);
    this.gzProcess = null;
    this.simPid = null;
    throw new Error(`Failed to launch simulation: ${err.message}`);
  }
}

async function exitSimulation() {
  if (!this.simPid) {
    throw new Error("No simulation running.");
  }

  try {
    process.kill(-this.simPid, "SIGTERM");
    console.log(`Terminated simulation (pid: ${this.simPid})`);
    this.simPid = null;
    this.gzProcess = null;
    return "Simulation exited.";
  } catch (err) {
    console.error("Failed to exit simulation:", err.message);
    throw new Error("Failed to exit simulation.");
  }
}

module.exports = {
  launchSimulation,
  exitSimulation
};
