const { spawn } = require("child_process");

let ros2Publisher = null;

async function publishMessage(input, node) {
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

async function sendRos2Cmd(input) {
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
  
  return new Promise((resolve, reject) => {
    const proc = spawn(parts[0], parts.slice(1));

    let stdout = "", stderr = "";

    proc.stdout.on("data", (data) => stdout += data.toString());
    proc.stderr.on("data", (data) => stderr += data.toString());

    proc.on("close", (code) => {
    console.log("[WoT Action] Sending ros2 cmd: ", commandStr);
      const cleanOutput = stdout.trim();
      const lines = cleanOutput.split("\n");

      if (code === 0) {
        resolve({ lines, raw: cleanOutput });
      } else {
        reject(new Error(`❌ ROS 2 command failed:\n${stderr || cleanOutput}`));
      }
    });

  });
}

module.exports = {
  publishMessage,
  sendRos2Cmd
};
