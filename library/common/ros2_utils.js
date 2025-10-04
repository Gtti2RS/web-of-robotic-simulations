const { spawn } = require('child_process');

let ros2Publisher = null;
let activeChildProcesses = new Set(); // Track active child processes

function makePublishMessage(node) {
  return async function publishMessage(input) {
    const msg = await input.value();

    if (typeof msg !== "string") {
      throw new Error("Input must be a string.");
    }

    if (!ros2Publisher) {
      ros2Publisher = node.createPublisher(
        "std_msgs/msg/String",
        "wot_topic"
      );
    }

    try {
      ros2Publisher.publish({ data: msg });
      console.log("[WoT Action] Published message: ", msg);
      return `Published: ${msg}`;
    } catch (err) {
      throw new Error("Failed to publish message: " + err.message);
    }
  };
}


function parseCommand(cmdStr) {
  const regex = /'[^']*'|"[^"]*"|\S+/g;
  return cmdStr.match(regex)?.map(arg => {
    if ((arg.startsWith('"') && arg.endsWith('"')) || (arg.startsWith("'") && arg.endsWith("'"))) {
      return arg.slice(1, -1);
    }
    return arg;
  }) || [];
}

function makeSendRos2Cmd(node, { timeoutMs = 10000 } = {}) {
  return async function sendRos2CmdAction(input) {
    const commandStr = await input.value();

    if (typeof commandStr !== "string" || !commandStr.trim().startsWith("ros2 ")) {
      throw new Error("Only 'ros2' commands are allowed.");
    }

    const forbidden = [";", "&&", "|", "`", "$(", "<", ">", "\\"];
    if (forbidden.some(f => commandStr.includes(f))) {
      throw new Error("Unsafe characters detected in command.");
    }

    const parts = parseCommand(commandStr);
    if (parts[0] !== "ros2") {
      throw new Error("Command must start with 'ros2'.");
    }

    try {
      const result = await new Promise((resolve, reject) => {
        // Spawn the ROS 2 command as a child process
        const child = spawn(parts[0], parts.slice(1), {
          stdio: ['pipe', 'pipe', 'pipe'],
          shell: false,
          env: { ...process.env }
        });

        // Track this child process
        activeChildProcesses.add(child);

        let stdout = '';
        let stderr = '';

        // Collect stdout
        child.stdout.on('data', (data) => {
          stdout += data.toString();
        });

        // Collect stderr
        child.stderr.on('data', (data) => {
          stderr += data.toString();
        });

        // Handle process completion
        child.on('close', (code) => {
          // Remove from tracking when process completes
          activeChildProcesses.delete(child);
          
          if (code === 0) {
            resolve({
              return_code: 0,
              stdout: stdout.trim(),
              stderr: stderr.trim()
            });
          } else {
            resolve({
              return_code: code,
              stdout: stdout.trim(),
              stderr: stderr.trim()
            });
          }
        });

        // Handle process errors
        child.on('error', (error) => {
          // Remove from tracking on error
          activeChildProcesses.delete(child);
          reject(new Error(`Failed to spawn process: ${error.message}`));
        });

        // Set timeout with improved signal handling
        const timeout = setTimeout(() => {
          console.log(`[WoT Action] Command timed out, terminating process PID: ${child.pid}`);
          
          // Try SIGTERM first
          child.kill('SIGTERM');
          
          // If still alive after 2 seconds, force kill with SIGKILL
          const forceKillTimeout = setTimeout(() => {
            if (!child.killed) {
              console.log(`[WoT Action] Force killing stuck process PID: ${child.pid}`);
              child.kill('SIGKILL');
            }
          }, 2000);
          
          // Clean up force kill timeout when process exits
          child.on('close', () => {
            clearTimeout(forceKillTimeout);
          });
          
          reject(new Error(`Command timed out after ${timeoutMs}ms`));
        }, timeoutMs);

        child.on('close', () => {
          clearTimeout(timeout);
        });
      });

      if (result.return_code === 0) {
        console.log(`[WoT Action] Executed ros2 command: ${commandStr}`);
        const cleanOutput = result.stdout || '';
        const lines = cleanOutput ? cleanOutput.split("\n") : [];
        return { lines, raw: cleanOutput };
      } else {
        const errorMsg = result.stderr || result.stdout || 'Unknown error occurred';
        console.error(`[WoT Action] ROS 2 command failed: ${errorMsg}`);
        throw new Error(` ROS 2 command failed: ${errorMsg}`);
      }
    } catch (error) {
      console.error(`[WoT Action] Process execution failed: ${error.message}`);
      throw new Error(` Process execution failed: ${error.message}`);
    }
  };
}


module.exports = {
  makePublishMessage,
  makeSendRos2Cmd
};
