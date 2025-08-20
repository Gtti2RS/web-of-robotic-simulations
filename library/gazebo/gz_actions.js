const { spawn, exec } = require("child_process");
const { resolveFilePath, saveFile } = require("../common/fileUtils");
const path = require('path');

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

async function getModelList() {
  return new Promise((resolve, reject) => {
    const world = cachedWorldNames.length > 0 ? cachedWorldNames[0] : null;
    const cmd = `gz service -s /world/${world}/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --req ''`;

    exec(cmd, (error, stdout, stderr) => {
      if (error) return reject(stderr || error.message);

      const models = [];
      const modelRegex = /model\s*{\s*name:\s*"([^"]+)"[^}]*?id:\s*(\d+)/g;
      let match;
      while ((match = modelRegex.exec(stdout)) !== null) {
        models.push({ name: match[1], id: parseInt(match[2]) });
      }
      resolve(models);
    });
  });
}

async function entity_management(params) {
  const {
    action, name, filename, x = 0.0, y = 0.0, z = 0.0,
    zRot = 0.0, wRot = 1.0, world = cachedWorldNames.length > 0 ? cachedWorldNames[0] : null
  } = await params.value();

  const timeout = 1000;
  let cmd = '';
  let fullPath = '';

  try {
    if (!world) {
      throw new Error("No world has been loaded.")
    }
    if (action === 'spawn') {
      if (!filename) throw new Error("Missing filename.");
      fullPath = await resolveFilePath(filename);
      if (!fullPath) {
        throw new Error(`File not found: ${filename}`);
      }

      cmd = `gz service -s /world/${world}/create \
--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout ${timeout} \
--req 'sdf_filename: "${fullPath}", name: "${name}", pose: { position: { x: ${x}, y: ${y}, z: ${z} } }'`;
    }

    else if (action === 'move') {
      cmd = `gz service -s /world/${world}/set_pose \
--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout ${timeout} \
--req 'name: "${name}", position: { x: ${x}, y: ${y}, z: ${z} }, orientation: { z: ${zRot}, w: ${wRot} }'`;
    }

    else if (action === 'remove') {
      cmd = `gz service -s /world/${world}/remove \
--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout ${timeout} \
--req 'name: "${name}", type: 2'`;
    }

    else {
      throw new Error(`Unsupported action type: "${action}".`);
    }

    return new Promise((resolve, reject) => {
      console.debug("[Entity_management]Executing: ", cmd);
      exec(cmd, (err) => {
        if (err) {
          console.error(`[entity_management error] ${err.message}`);
          return reject(`Error: ${err.message}`);
        }
        console.debug("[Entity_management]Action completed successfully.");
        resolve("Action completed successfully.");
      });
    });

  } catch (err) {
    console.error(`[entity_management exception] ${err.message}`);
    throw new Error(`Error: ${err.message}`);
  }
}

async function save_world(input) {
  const { name } = await input.value();

  if (!cachedWorldNames.length) {
    console.error('[save_world] No active world');
    throw new Error('No active Gazebo world found.');
  }

  const worldName = cachedWorldNames[0];
  const service = `/world/${worldName}/generate_world_sdf`;

  const gzCommand = [
    'gz service',
    `-s ${service}`,
    '--reqtype gz.msgs.SdfGeneratorConfig',
    '--reptype gz.msgs.StringMsg',
    '--req'
  ].join(' ');

  return new Promise((resolve, reject) => {
    exec(gzCommand, async (error, stdout, stderr) => {
      if (error) {
        console.error('[save_world error]', stderr || error.message);
        return reject(new Error('Failed to call Gazebo service'));
      }

      const line = stdout.trim();
      if (!line.startsWith('data:')) {
        return reject(new Error('Unexpected service output'));
      }

      try {
        const jsonLine = `{${line.replace(/^data:\s*/, '"data": ').replace(/'/g, '"')}}`;
        const parsed = JSON.parse(jsonLine);
        const sdfString = parsed.data;

        const safeName = name && name.trim() !== ''
          ? name.trim().replace(/[^a-zA-Z0-9_\-]/g, '_') + '.sdf'
          : `world_${worldName}_${new Date().toISOString().replace(/[:.]/g, '-')}.sdf`;

        const filePath = path.join(__dirname, '../../saved/world', safeName);
        const finalSdf = `<?xml version="1.0" ?>\n${sdfString}`;
        await saveFile(filePath, finalSdf);

        console.log(`World '${worldName}' saved to '${safeName}'`);
        resolve(`World saved as '${safeName}'`);
      } catch (parseErr) {
        console.error('[save_world] JSON parse error:', parseErr.message);
        reject(new Error('Failed to parse Gazebo service response'));
      }
    });
  });
}



module.exports = {
  launchSimulation,
  exitSimulation,
  sim_control,
  getModelList,
  entity_management,
  save_world
};
