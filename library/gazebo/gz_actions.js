const { spawn, exec } = require("child_process");
const { resolveFilePath } = require("../common/fileUtils");

let cachedWorldNames = [];

function detectWorldNames() {
  return new Promise((resolve, reject) => {
    exec('gz topic -l', (err, stdout) => {
      if (err) return reject(new Error('Failed to list topics'));

      const worldSet = new Set();
      stdout.split('\n').forEach(line => {
        const match = line.match(/^\/world\/([^\/]+)/);
        if (match) worldSet.add(match[1]);
      });

      cachedWorldNames = [...worldSet];
      if (cachedWorldNames.length === 0) {
        return reject(new Error('No /world/<name>/ topics found'));
      }

      console.log('[World Detection] Worlds found:', cachedWorldNames);
      resolve(cachedWorldNames);
    });
  });
}

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
    await new Promise(resolve => setTimeout(resolve, 1500)); // tune as needed

    try {
      await detectWorldNames();
      return `Simulation launched: ${fullPath}, worlds: ${cachedWorldNames.join(', ')}`;
    } catch (detectErr) {
      console.warn('[launchSimulation] World detection failed:', detectErr.message);
      return `Simulation launched: ${fullPath}, but world detection failed`;
    }
  } catch (err) {
    console.error('[launchSimulation error]', err);
    throw new Error('Simulation launch failed: ' + err.message);
  }
}

async function exitSimulation() {
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
}

async function sim_control(input) {
  try {
    const { mode } = await input.value();
    const world = cachedWorldNames.length > 0 ? cachedWorldNames[0] : null;
    if (!world) throw new Error('No worldName available and none was cached');

    let req;
    switch (mode) {
      case 'pause': req = "'pause: true'"; break;
      case 'run':
      case 'resume': req = "'pause: false'"; break;
      case 'reset': req = "'reset: {all: true}, pause: true'"; break;
      default: throw new Error(`Invalid mode: ${mode}`);
    }

    const cmd = `gz service -s /world/${world}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req ${req}`;
    console.log('[sim_control]', cmd);

    return await new Promise((resolve, reject) => {
      exec(cmd, (err, stdout, stderr) => {
        if (err) {
          console.error('[sim_control error]', stderr || err.message);
          return reject(new Error(`Failed to ${mode} simulation: ${stderr || err.message}`));
        }

        // Instead of raw "data: true", return readable message
        resolve(`Simulation "${world}" successfully ${mode}d.`);
      });
    });
  } catch (err) {
    console.error('[sim_control internal error]', err.message);
    throw new Error(`[sim_control] ${err.message}`);
  }
}


module.exports = {
  launchSimulation,
  exitSimulation,
  sim_control
};
