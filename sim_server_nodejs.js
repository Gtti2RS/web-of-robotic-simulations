const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const Servient = require('@node-wot/core').Servient;
const HttpServer = require('@node-wot/binding-http').HttpServer;
const rclnodejs = require('rclnodejs');
const { startRosLaunch, stopRosLaunch } = require('./ros2_launch_helper');

const WORLD_DIR = path.join(__dirname, 'worlds');
let bridgeProcess = null;

(async () => {
  await rclnodejs.init();
  const node = new rclnodejs.Node('sim_server_node');
  setInterval(() => rclnodejs.spinOnce(node), 100); // periodically spin the node

  const pauseClient = node.createClient('std_srvs/srv/SetBool', '/world/default/set_physics_paused');
  const resetClient = node.createClient('std_srvs/srv/Empty', '/world/default/reset');
  const stepClient = node.createClient('std_srvs/srv/Empty', '/world/default/run_once');

  const callPauseService = async (pause) => {
    return new Promise((resolve, reject) => {
      if (!pauseClient.waitForService(1000)) {
        return reject('Pause service not available');
      }
      pauseClient.sendRequest({ data: pause }, (response) => {
        response.success ? resolve(response.message) : reject(response.message);
      });
    });
  };

  const callEmptyService = async (client) => {
    return new Promise((resolve, reject) => {
      if (!client.waitForService(1000)) {
        return reject('Service not available');
      }
      client.sendRequest({}, () => resolve("OK"));
    });
  };

  const servient = new Servient();
  servient.addServer(new HttpServer({ port: 8080 }));

  servient.start().then(async (WoT) => {
    const td = JSON.parse(fs.readFileSync(path.join(__dirname, 'sim.json'), 'utf-8'));
    const thing = await WoT.produce(td);

    thing.setPropertyReadHandler('world_list', async () => {
      return fs.readdirSync(WORLD_DIR).filter(f => f.endsWith('.sdf') || f.endsWith('.world'));
    });

    thing.setActionHandler('launch', async (param) => {
      const worldFile = await param.value();
      const worldPath = path.join(WORLD_DIR, worldFile);
      if (!fs.existsSync(worldPath)) {
        throw new Error(`World file not found: ${worldFile}`);
      }
      if (bridgeProcess) stopRosLaunch(bridgeProcess);
      bridgeProcess = startRosLaunch('my_package', 'ros2_launch_bridge.py', [`world:=${worldPath}`]);
      return `Launched simulation with: ${worldFile}`;
    });

    thing.setActionHandler('exit', async () => {
      if (bridgeProcess) {
        stopRosLaunch(bridgeProcess);
        bridgeProcess = null;
        return 'Simulation exited';
      } else {
        return 'No active simulation';
      }
    });

    thing.setActionHandler('pause', async () => {
      if (!bridgeProcess) return 'Simulation not running';
      try {
        const result = await callPauseService(true);
        return `Paused: ${result}`;
      } catch (e) {
        return `Pause failed: ${e}`;
      }
    });

    thing.setActionHandler('run', async () => {
      if (!bridgeProcess) return 'Simulation not running';
      try {
        const result = await callPauseService(false);
        return `Resumed: ${result}`;
      } catch (e) {
        return `Run failed: ${e}`;
      }
    });

    thing.setActionHandler('reset', async () => {
      if (!bridgeProcess) return 'Simulation not running';
      try {
        await callEmptyService(resetClient);
        return 'Simulation reset';
      } catch (e) {
        return `Reset failed: ${e}`;
      }
    });

    thing.setActionHandler('step', async (param) => {
      if (!bridgeProcess) return 'Simulation not running';
      const steps = parseInt(await param.value());
      if (isNaN(steps) || steps < 1) return 'Invalid step count';
      try {
        for (let i = 0; i < steps; i++) {
          await callEmptyService(stepClient);
        }
        return `Simulation stepped ${steps} times`;
      } catch (e) {
        return `Step failed: ${e}`;
      }
    });

    thing.expose().then(() => {
      console.log(`${thing.getThingDescription().title} ready`);
    });
  });
})();
