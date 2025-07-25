const { spawn } = require('child_process');

function startRosLaunch(packageName, launchFile, args = []) {
  const process = spawn('ros2', ['launch', packageName, launchFile, ...args], {
    detached: true,
    stdio: 'ignore'
  });
  process.unref();
  return process;
}

function stopRosLaunch(processRef) {
  if (processRef && processRef.kill) {
    try {
      processRef.kill();
    } catch (e) {
      console.warn('Failed to stop ROS 2 launch process:', e.message);
    }
  }
}

module.exports = { startRosLaunch, stopRosLaunch };
