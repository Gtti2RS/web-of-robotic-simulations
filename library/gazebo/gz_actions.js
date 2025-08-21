const { spawn, exec } = require("child_process");
const { resolveFilePath, saveFile } = require("../common/fileUtils");
const path = require('path');
const { deg2quat } = require("../common/deg2quat");

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

function get_world() {
  const world = cachedWorldNames.length > 0 ? cachedWorldNames[0] : null;
  if (!world) {
    throw new Error('No active world.');
  }
  else {
    return world;
  }
}

async function getEntities() {
  const world = get_world();

  const cmd = `gz service -s /world/${world}/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --req ''`;

  const stdout = await new Promise((resolve, reject) => {
    exec(cmd, (error, out, stderr) => error ? reject(new Error(stderr || error.message)) : resolve(out));
  });

  // Parse in one pass using lightweight regexes
  const models = [];
  const blockRe = /model\s*{([\s\S]*?)\n}/g;
  let block;
  while ((block = blockRe.exec(stdout)) !== null) {
    const t = block[1];

    const name = (t.match(/name:\s*"([^"]+)"/) || [])[1];
    if (!name) continue;

    const id = parseInt((t.match(/\bid:\s*([0-9]+)/) || [])[1] || '0', 10) || undefined;

    const pos = t.match(/position\s*{[\s\S]*?x:\s*([-+0-9.eE]+)[\s\S]*?y:\s*([-+0-9.eE]+)[\s\S]*?z:\s*([-+0-9.eE]+)/);
    const position = pos ? { x: +pos[1], y: +pos[2], z: +pos[3] } : { x: 0, y: 0, z: 0 };

    const ori = t.match(/orientation\s*{[\s\S]*?x:\s*([-+0-9.eE]+)[\s\S]*?y:\s*([-+0-9.eE]+)[\s\S]*?z:\s*([-+0-9.eE]+)[\s\S]*?w:\s*([-+0-9.eE]+)/);
    const orientation = ori ? { x: +ori[1], y: +ori[2], z: +ori[3], w: +ori[4] } : { x: 0, y: 0, z: 0, w: 1 };

    models.push({ name, id, pose: { position, orientation } });
  }

  return models;
}

async function read_entity_info() {
  try {
    const models = await getEntities();
    return models;
  } catch (err) {
    console.error('[available_models read]', err);
    return `Failed to read available models: ${err.message || String(err)}`;
  }
}

async function entityExists(name) {
  const models = await getEntities();
  return models.some(m => m.name === name);
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
    cachedWorldNames = [];
    return "Simulation exited.";
  } else {
    return "No simulation is currently running.";
  }
}

async function sim_control(input) {
  try {
    const world = get_world();
    const { mode } = await input.value();

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

async function entity_management(params) {
  const data = await params.value();

  const action = data.action;
  const name = data.name;
  const filename = data.filename;
  const { x, y, z } = data.position;
  const { qx, qy, qz, qw } = deg2quat(data.orientation);
  const world = cachedWorldNames.length > 0 ? cachedWorldNames[0] : null;

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
--req 'sdf_filename: "${fullPath}", name: "${name}", pose: { position: { x: ${x}, y: ${y}, z: ${z} }, orientation: { x: ${qx}, y: ${qy}, z: ${qz}, w: ${qw}} }'`;
    }

    else if (action === 'move') {
      cmd = `gz service -s /world/${world}/set_pose \
--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout ${timeout} \
--req 'name: "${name}", position: { x: ${x}, y: ${y}, z: ${z} }, orientation: { x: ${qx}, y: ${qy}, z: ${qz}, w: ${qw}}'`;
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

async function remove_entity(input) {
  try {
    const world = get_world();
    const data = await input.value();
    const name = data.name;

    if (!name) return 'Entity name is required.';

    const exists = await entityExists(name);
    if (!exists) return `Entity ${name} not found.`;

    const req = `name: "${name}", type: 2`;
    const cmd = `gz service -s /world/${world}/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 --req '${req}'`;

    await new Promise((resolve, reject) => {
      exec(cmd, (error, stderr) => error ? reject(new Error(stderr || error.message)) : resolve());
    });

    return `${name} has been removed.`;

  } catch (err) {
    console.error('[remove_entity error]', err);
    return `Failed to remove entity: ${err.message || String(err)}`;
  }
}

async function save_world(input) {
  const worldName = get_world();
  const { name } = await input.value();

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
  entity_management,
  save_world,
  remove_entity,
  read_entity_info
};
