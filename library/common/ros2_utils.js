const { callService } = require('./ros2_service_helper');

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
      const { resp } = await callService(
        node,
        {
          srvType: 'sim_process_supervisor_interfaces/srv/ExecCmd',
          serviceName: '/process/exec',
          payload: {
            cmd: commandStr
          }
        },
        { timeoutMs }
      );

      if (resp?.return_code === 0) {
        console.log(`[WoT Action] Executed ros2 command: ${commandStr}`);
        const cleanOutput = resp.stdout?.trim() || '';
        const lines = cleanOutput ? cleanOutput.split("\n") : [];
        return { lines, raw: cleanOutput };
      } else {
        const errorMsg = resp?.stderr || resp?.error || resp?.message || 'Unknown error occurred';
        console.error(`[WoT Action] ROS 2 command failed: ${errorMsg}`);
        throw new Error(` ROS 2 command failed: ${errorMsg}`);
      }
    } catch (error) {
      console.error(`[WoT Action] Service call failed: ${error.message}`);
      throw new Error(` Service call failed: ${error.message}`);
    }
  };
}

module.exports = {
  publishMessage,
  makeSendRos2Cmd
};
