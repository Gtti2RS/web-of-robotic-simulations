#!/usr/bin/env node
// An interactive script to test the affordaces in case postman is down
const readline = require('readline');
const https = require('https');
const http = require('http');

// Configuration
const CONFIG = {
  gz_controller: {
    baseUrl: 'http://localhost:8080/gz_controller',
    name: 'Gazebo Controller',
    simulator: 'Gazebo'
  },
  cs_controller: {
    baseUrl: 'http://localhost:8081/cs_controller',
    name: 'CoppeliaSim Controller',
    simulator: 'CoppeliaSim'
  },
  ur10_gazebo: {
    baseUrl: 'http://localhost:8083/ur10_server',
    name: 'UR10 Robot (Gazebo)',
    simulator: 'Gazebo'
  },
  ur10_coppelia: {
    baseUrl: 'http://localhost:8084/ur10_server',
    name: 'UR10 Robot (CoppeliaSim)',
    simulator: 'CoppeliaSim'
  }
};

// Create readline interface
const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

// Utility function to make HTTP requests
function makeRequest(url, method = 'GET', data = null) {
  return new Promise((resolve, reject) => {
    const urlObj = new URL(url);
    const options = {
      hostname: urlObj.hostname,
      port: urlObj.port,
      path: urlObj.pathname + urlObj.search,
      method: method,
      headers: {
        'Content-Type': 'application/json'
      }
    };

    if (data) {
      const jsonData = JSON.stringify(data);
      options.headers['Content-Length'] = Buffer.byteLength(jsonData);
    }

    const req = (urlObj.protocol === 'https:' ? https : http).request(options, (res) => {
      let responseData = '';
      res.on('data', (chunk) => {
        responseData += chunk;
      });
      res.on('end', () => {
        try {
          const parsed = JSON.parse(responseData);
          resolve({ status: res.statusCode, data: parsed });
        } catch (e) {
          resolve({ status: res.statusCode, data: responseData });
        }
      });
    });

    req.on('error', (err) => {
      reject(err);
    });

    if (data) {
      req.write(JSON.stringify(data));
    }
    req.end();
  });
}

// Operation definitions
const OPERATIONS = {
  // Gazebo Controller Operations
  'gz_scene_load': {
    name: 'Load Scene',
    service: 'gz_controller',
    action: 'manageScene',
    input: { mode: 'load', fileName: '' },
    description: 'Load a Gazebo scene file'
  },
  'gz_scene_save': {
    name: 'Save Scene',
    service: 'gz_controller',
    action: 'manageScene',
    input: { mode: 'save', fileName: '' },
    description: 'Save current Gazebo scene'
  },
  'gz_scene_close': {
    name: 'Close Scene',
    service: 'gz_controller',
    action: 'manageScene',
    input: { mode: 'close' },
    description: 'Close current scene and load empty world'
  },
  'gz_model_load': {
    name: 'Load Model',
    service: 'gz_controller',
    action: 'manageModel',
    input: { mode: 'load', fileName: '', position: { x: 0, y: 0, z: 0 }, orientation: { roll: 0, pitch: 0, yaw: 0 } },
    description: 'Load a model into the scene'
  },
  'gz_model_remove': {
    name: 'Remove Model',
    service: 'gz_controller',
    action: 'manageModel',
    input: { mode: 'remove', modelName: '' },
    description: 'Remove a model from the scene'
  },
  'gz_model_setpose': {
    name: 'Set Model Pose',
    service: 'gz_controller',
    action: 'manageModel',
    input: { mode: 'setPose', id: '', position: { x: 0, y: 0, z: 0 }, orientation: { roll: 0, pitch: 0, yaw: 0 } },
    description: 'Set position and orientation of a model'
  },
  'gz_sim_pause': {
    name: 'Pause Simulation',
    service: 'gz_controller',
    action: 'simControl',
    input: { mode: 'pause' },
    description: 'Pause the simulation'
  },
  'gz_sim_run': {
    name: 'Run Simulation',
    service: 'gz_controller',
    action: 'simControl',
    input: { mode: 'run' },
    description: 'Start/resume the simulation'
  },
  'gz_sim_stop': {
    name: 'Stop Simulation',
    service: 'gz_controller',
    action: 'simControl',
    input: { mode: 'stop' },
    description: 'Stop the simulation'
  },
  'gz_sim_faster': {
    name: 'Increase Simulation Speed',
    service: 'gz_controller',
    action: 'simControl',
    input: { mode: 'faster' },
    description: 'Increase real-time factor by 0.1'
  },
  'gz_sim_slower': {
    name: 'Decrease Simulation Speed',
    service: 'gz_controller',
    action: 'simControl',
    input: { mode: 'slower' },
    description: 'Decrease real-time factor by 0.1'
  },
  'gz_ros_cmd': {
    name: 'Send ROS2 Command',
    service: 'gz_controller',
    action: 'sendRos2Cmd',
    input: '',
    description: 'Execute a ROS2 CLI command'
  },
  'gz_publish_msg': {
    name: 'Publish ROS2 Message',
    service: 'gz_controller',
    action: 'publishMessage',
    input: '',
    description: 'Publish a string message to ROS2'
  },
  'gz_get_assets': {
    name: 'Get Available Assets',
    service: 'gz_controller',
    action: 'properties/assets',
    input: null,
    description: 'List all available Gazebo assets'
  },
  'gz_get_stats': {
    name: 'Get Simulation Statistics',
    service: 'gz_controller',
    action: 'properties/simStats',
    input: null,
    description: 'Get current simulation statistics'
  },
  'gz_get_models': {
    name: 'Get Current Models',
    service: 'gz_controller',
    action: 'properties/models',
    input: null,
    description: 'Get list of models in the scene'
  },
  'gz_get_poses': {
    name: 'Get Entity Poses',
    service: 'gz_controller',
    action: 'properties/poses',
    input: null,
    description: 'Get current poses of all entities'
  },
  'gz_get_visualize': {
    name: 'Get Visualization Status',
    service: 'gz_controller',
    action: 'properties/visualize',
    input: null,
    description: 'Get camera visualization status'
  },
  'gz_set_visualize': {
    name: 'Set Visualization Status',
    service: 'gz_controller',
    action: 'properties/visualize',
    input: true,
    description: 'Enable/disable camera visualization'
  },

  // CoppeliaSim Controller Operations (placeholder for future implementation)
  'cs_scene_load': {
    name: 'Load Scene (CoppeliaSim)',
    service: 'cs_controller',
    action: 'manageScene',
    input: { mode: 'load', fileName: '' },
    description: 'Load a CoppeliaSim scene file'
  },
  'cs_scene_save': {
    name: 'Save Scene (CoppeliaSim)',
    service: 'cs_controller',
    action: 'manageScene',
    input: { mode: 'save', fileName: '' },
    description: 'Save current CoppeliaSim scene'
  },
  'cs_scene_close': {
    name: 'Close Scene (CoppeliaSim)',
    service: 'cs_controller',
    action: 'manageScene',
    input: { mode: 'close' },
    description: 'Close current scene and load empty world'
  },
  'cs_model_load': {
    name: 'Load Model (CoppeliaSim)',
    service: 'cs_controller',
    action: 'manageModel',
    input: { mode: 'load', fileName: '', position: { x: 0, y: 0, z: 0 }, orientation: { roll: 0, pitch: 0, yaw: 0 } },
    description: 'Load a model into the CoppeliaSim scene'
  },
  'cs_model_remove': {
    name: 'Remove Model (CoppeliaSim)',
    service: 'cs_controller',
    action: 'manageModel',
    input: { mode: 'remove', modelName: '' },
    description: 'Remove a model from the CoppeliaSim scene'
  },
  'cs_model_setpose': {
    name: 'Set Model Pose (CoppeliaSim)',
    service: 'cs_controller',
    action: 'manageModel',
    input: { mode: 'setPose', id: '', position: { x: 0, y: 0, z: 0 }, orientation: { roll: 0, pitch: 0, yaw: 0 } },
    description: 'Set position and orientation of a model in CoppeliaSim'
  },
  'cs_sim_pause': {
    name: 'Pause Simulation (CoppeliaSim)',
    service: 'cs_controller',
    action: 'simControl',
    input: { mode: 'pause' },
    description: 'Pause the CoppeliaSim simulation'
  },
  'cs_sim_run': {
    name: 'Run Simulation (CoppeliaSim)',
    service: 'cs_controller',
    action: 'simControl',
    input: { mode: 'run' },
    description: 'Start/resume the CoppeliaSim simulation'
  },
  'cs_sim_stop': {
    name: 'Stop Simulation (CoppeliaSim)',
    service: 'cs_controller',
    action: 'simControl',
    input: { mode: 'stop' },
    description: 'Stop the CoppeliaSim simulation'
  },
  'cs_get_assets': {
    name: 'Get Available Assets (CoppeliaSim)',
    service: 'cs_controller',
    action: 'properties/assets',
    input: null,
    description: 'List all available CoppeliaSim assets'
  },
  'cs_get_stats': {
    name: 'Get Simulation Statistics (CoppeliaSim)',
    service: 'cs_controller',
    action: 'properties/simStats',
    input: null,
    description: 'Get current CoppeliaSim simulation statistics'
  },
  'cs_get_models': {
    name: 'Get Current Models (CoppeliaSim)',
    service: 'cs_controller',
    action: 'properties/models',
    input: null,
    description: 'Get list of models in the CoppeliaSim scene'
  },
  'cs_get_poses': {
    name: 'Get Entity Poses (CoppeliaSim)',
    service: 'cs_controller',
    action: 'properties/poses',
    input: null,
    description: 'Get current poses of all entities in CoppeliaSim'
  },

  // UR10 Robot Operations (Gazebo)
  'ur10_gz_joint_positions': {
    name: 'Get Joint Positions (Gazebo)',
    service: 'ur10_gazebo',
    action: 'properties/jointPositions',
    input: null,
    description: 'Get current joint positions'
  },
  'ur10_gz_move_joint': {
    name: 'Move to Joint Positions (Gazebo)',
    service: 'ur10_gazebo',
    action: 'moveToJoint',
    input: {
      shoulder_pan_joint: null,
      shoulder_lift_joint: null,
      elbow_joint: null,
      wrist_1_joint: null,
      wrist_2_joint: null,
      wrist_3_joint: null
    },
    description: 'Move robot to specified joint positions'
  },
  'ur10_gz_move_cartesian': {
    name: 'Move to Cartesian Position (Gazebo)',
    service: 'ur10_gazebo',
    action: 'moveToCartesian',
    input: {
      position: { x: 0, y: 0, z: 0 },
      orientation: { roll: 0, pitch: 0, yaw: 0 }
    },
    description: 'Move robot to Cartesian position using IK'
  },
  'ur10_gz_grip_open': {
    name: 'Open Gripper (Gazebo)',
    service: 'ur10_gazebo',
    action: 'gripOpen',
    input: null,
    description: 'Open the RG2 gripper'
  },
  'ur10_gz_grip_close': {
    name: 'Close Gripper (Gazebo)',
    service: 'ur10_gazebo',
    action: 'gripClose',
    input: null,
    description: 'Close the RG2 gripper'
  },
  'ur10_gz_emergency_stop': {
    name: 'Emergency Stop (Gazebo)',
    service: 'ur10_gazebo',
    action: 'emergencyStop',
    input: {},
    description: 'Immediately stop all robot movement'
  },

  // UR10 Robot Operations (CoppeliaSim)
  'ur10_cp_joint_positions': {
    name: 'Get Joint Positions (CoppeliaSim)',
    service: 'ur10_coppelia',
    action: 'properties/jointPositions',
    input: null,
    description: 'Get current joint positions'
  },
  'ur10_cp_move_joint': {
    name: 'Move to Joint Positions (CoppeliaSim)',
    service: 'ur10_coppelia',
    action: 'moveToJoint',
    input: {
      shoulder_pan_joint: null,
      shoulder_lift_joint: null,
      elbow_joint: null,
      wrist_1_joint: null,
      wrist_2_joint: null,
      wrist_3_joint: null
    },
    description: 'Move robot to specified joint positions'
  },
  'ur10_cp_move_cartesian': {
    name: 'Move to Cartesian Position (CoppeliaSim)',
    service: 'ur10_coppelia',
    action: 'moveToCartesian',
    input: {
      position: { x: 0, y: 0, z: 0 },
      orientation: { roll: 0, pitch: 0, yaw: 0 }
    },
    description: 'Move robot to Cartesian position using IK'
  },
  'ur10_cp_grip_open': {
    name: 'Open Gripper (CoppeliaSim)',
    service: 'ur10_coppelia',
    action: 'gripOpen',
    input: null,
    description: 'Open the RG2 gripper'
  },
  'ur10_cp_grip_close': {
    name: 'Close Gripper (CoppeliaSim)',
    service: 'ur10_coppelia',
    action: 'gripClose',
    input: null,
    description: 'Close the RG2 gripper'
  },
  'ur10_cp_emergency_stop': {
    name: 'Emergency Stop (CoppeliaSim)',
    service: 'ur10_coppelia',
    action: 'emergencyStop',
    input: {},
    description: 'Immediately stop all robot movement'
  }
};

// Helper function to get user input
function askQuestion(question) {
  return new Promise((resolve) => {
    rl.question(question, (answer) => {
      resolve(answer.trim());
    });
  });
}

// Helper function to get numeric input
async function askNumericInput(question, defaultValue = 0, unit = '') {
  while (true) {
    const input = await askQuestion(question);
    if (input === '') return defaultValue;
    const num = parseFloat(input);
    if (!isNaN(num)) return num;
    console.log(`Please enter a valid number${unit ? ` in ${unit}` : ''}.`);
  }
}

// Helper function to get boolean input
async function askBooleanInput(question, defaultValue = true) {
  while (true) {
    const input = await askQuestion(question);
    if (input === '') return defaultValue;
    const lower = input.toLowerCase();
    if (['true', 't', 'yes', 'y', '1'].includes(lower)) return true;
    if (['false', 'f', 'no', 'n', '0'].includes(lower)) return false;
    console.log('Please enter true/false, yes/no, or 1/0.');
  }
}

// Function to collect input for an operation
async function collectOperationInput(operation) {
  const input = { ...operation.input };
  
  if (operation.action === 'manageScene') {
    if (operation.input.mode === 'load' || operation.input.mode === 'save') {
      input.fileName = await askQuestion('Enter filename (e.g., empty.sdf): ');
    }
  } else if (operation.action === 'manageModel') {
    if (operation.input.mode === 'load') {
      input.fileName = await askQuestion('Enter model filename (e.g., ur10_rg2_gazebo.urdf): ');
    } else if (operation.input.mode === 'remove') {
      const identifier = await askQuestion('Enter model name or ID to remove: ');
      // Try to determine if it's an ID (numeric) or name (string)
      if (!isNaN(identifier) && identifier !== '') {
        input.id = identifier;
      } else {
        input.modelName = identifier;
      }
    } else if (operation.input.mode === 'setPose') {
      const identifier = await askQuestion('Enter model name or ID: ');
      // Try to determine if it's an ID (numeric) or name (string)
      if (!isNaN(identifier) && identifier !== '') {
        input.id = identifier;
      } else {
        input.id = identifier; // Use as name if not numeric
      }
    }
    
    if (operation.input.mode === 'load' || operation.input.mode === 'setPose') {
      console.log('Enter position coordinates (in meters):');
      input.position.x = await askNumericInput('X position (default: 0): ', 0, 'meters');
      input.position.y = await askNumericInput('Y position (default: 0): ', 0, 'meters');
      input.position.z = await askNumericInput('Z position (default: 0): ', 0, 'meters');
      
      console.log('Enter orientation (Euler angles in degrees):');
      input.orientation.roll = await askNumericInput('Roll (default: 0): ', 0, 'degrees');
      input.orientation.pitch = await askNumericInput('Pitch (default: 0): ', 0, 'degrees');
      input.orientation.yaw = await askNumericInput('Yaw (default: 0): ', 0, 'degrees');
    }
  } else if (operation.action === 'sendRos2Cmd') {
    return await askQuestion('Enter ROS2 command (e.g., "ros2 topic list"): ');
  } else if (operation.action === 'publishMessage') {
    return await askQuestion('Enter message to publish: ');
  } else if (operation.action === 'moveToJoint') {
    console.log('Enter joint positions in degrees (press Enter to keep current position):');
    console.log('Joint limits: shoulder_pan: -180° to 180°, shoulder_lift: -180° to 0°, elbow: -140° to 140°, wrists: -180° to 180°');
    
    const shoulder_pan = await askQuestion('Shoulder pan joint (default: keep current): ');
    input.shoulder_pan_joint = shoulder_pan === '' ? null : parseFloat(shoulder_pan);
    
    const shoulder_lift = await askQuestion('Shoulder lift joint (default: keep current): ');
    input.shoulder_lift_joint = shoulder_lift === '' ? null : parseFloat(shoulder_lift);
    
    const elbow = await askQuestion('Elbow joint (default: keep current): ');
    input.elbow_joint = elbow === '' ? null : parseFloat(elbow);
    
    const wrist_1 = await askQuestion('Wrist 1 joint (default: keep current): ');
    input.wrist_1_joint = wrist_1 === '' ? null : parseFloat(wrist_1);
    
    const wrist_2 = await askQuestion('Wrist 2 joint (default: keep current): ');
    input.wrist_2_joint = wrist_2 === '' ? null : parseFloat(wrist_2);
    
    const wrist_3 = await askQuestion('Wrist 3 joint (default: keep current): ');
    input.wrist_3_joint = wrist_3 === '' ? null : parseFloat(wrist_3);
  } else if (operation.action === 'moveToCartesian') {
    console.log('Enter Cartesian position (in meters):');
    input.position.x = await askNumericInput('X position: ', 0, 'meters');
    input.position.y = await askNumericInput('Y position: ', 0, 'meters');
    input.position.z = await askNumericInput('Z position: ', 0, 'meters');
    
    console.log('Enter orientation (Euler angles in degrees):');
    input.orientation.roll = await askNumericInput('Roll: ', 0, 'degrees');
    input.orientation.pitch = await askNumericInput('Pitch: ', 0, 'degrees');
    input.orientation.yaw = await askNumericInput('Yaw: ', 0, 'degrees');
  } else if (operation.action === 'properties/visualize' && typeof operation.input === 'boolean') {
    return await askBooleanInput('Enable visualization? (true/false): ', true);
  }
  
  return input;
}

// Function to execute an operation
async function executeOperation(operation) {
  try {
    console.log(`\nExecuting: ${operation.name}`);
    console.log(`Service: ${CONFIG[operation.service].name}`);
    console.log(`Description: ${operation.description}`);
    
    let input = null;
    if (operation.input !== null) {
      input = await collectOperationInput(operation);
    }
    
    const config = CONFIG[operation.service];
    let url = `${config.baseUrl}/actions/${operation.action}`;
    let method = 'POST';
    
    if (operation.action.startsWith('properties/')) {
      url = `${config.baseUrl}/${operation.action}`;
      method = input !== null ? 'PUT' : 'GET';
    }
    
    console.log(`\nMaking ${method} request to: ${url}`);
    if (input !== null) {
      console.log('Payload:', JSON.stringify(input, null, 2));
    }
    
    const response = await makeRequest(url, method, input);
    
    console.log(`\nResponse Status: ${response.status}`);
    console.log('Response Data:');
    console.log(JSON.stringify(response.data, null, 2));
    
  } catch (error) {
    console.error(`\nError executing operation: ${error.message}`);
  }
}

// Function to select simulator
async function selectSimulator() {
  console.log('\n' + '='.repeat(80));
  console.log('WoT API Interactive Client - Simulator Selection');
  console.log('='.repeat(80));
  console.log('1. Gazebo');
  console.log('2. CoppeliaSim');
  console.log('3. Both (All Services)');
  console.log('='.repeat(80));
  
  while (true) {
    const choice = await askQuestion('Select simulator (1-3): ');
    const choiceNum = parseInt(choice);
    
    if (choiceNum === 1) return 'gazebo';
    if (choiceNum === 2) return 'coppelia';
    if (choiceNum === 3) return 'both';
    
    console.log('Invalid choice. Please select 1, 2, or 3.');
  }
}

// Function to display menu
function displayMenu(selectedSimulator) {
  console.log('\n' + '='.repeat(80));
  console.log(`WoT API Interactive Client - ${selectedSimulator === 'both' ? 'All Services' : selectedSimulator === 'gazebo' ? 'Gazebo Services' : 'CoppeliaSim Services'}`);
  console.log('='.repeat(80));
  
  const categories = {};
  
  // Build categories based on selected simulator
  Object.entries(OPERATIONS).forEach(([key, op]) => {
    const serviceConfig = CONFIG[op.service];
    if (!serviceConfig) return;
    
    const simulator = serviceConfig.simulator;
    const shouldInclude = selectedSimulator === 'both' || 
                         (selectedSimulator === 'gazebo' && simulator === 'Gazebo') ||
                         (selectedSimulator === 'coppelia' && simulator === 'CoppeliaSim');
    
    if (shouldInclude) {
      if (!categories[serviceConfig.name]) {
        categories[serviceConfig.name] = [];
      }
      categories[serviceConfig.name].push({ key, ...op });
    }
  });
  
  let index = 1;
  const operationMap = {};
  
  Object.entries(categories).forEach(([category, operations]) => {
    console.log(`\n${category}:`);
    console.log('-'.repeat(category.length + 1));
    
    operations.forEach(op => {
      console.log(`${index.toString().padStart(2)}. ${op.name}`);
      operationMap[index] = op.key;
      index++;
    });
  });
  
  console.log(`\n${index.toString().padStart(2)}. Change Simulator`);
  console.log(`${index + 1}. Exit`);
  console.log('\n' + '='.repeat(80));
  
  return { operationMap, totalOptions: index + 1 };
}

// Main function
async function main() {
  console.log('Welcome to the WoT API Interactive Client!');
  console.log('This tool allows you to interact with Gazebo Controller, CoppeliaSim Controller, and UR10 Robot APIs.');
  
  let selectedSimulator = await selectSimulator();
  
  while (true) {
    const { operationMap, totalOptions } = displayMenu(selectedSimulator);
    const choice = await askQuestion(`\nSelect an operation (1-${totalOptions}): `);
    
    const choiceNum = parseInt(choice);
    
    if (choiceNum === totalOptions) {
      console.log('Goodbye!');
      break;
    }
    
    if (choiceNum === totalOptions - 1) {
      // Change simulator
      selectedSimulator = await selectSimulator();
      continue;
    }
    
    if (choiceNum >= 1 && choiceNum <= Object.keys(operationMap).length) {
      const operationKey = operationMap[choiceNum];
      const operation = OPERATIONS[operationKey];
      await executeOperation(operation);
      
      const continueChoice = await askQuestion('\nPress Enter to continue or type "exit" to quit: ');
      if (continueChoice.toLowerCase() === 'exit') {
        console.log('Goodbye!');
        break;
      }
    } else {
      console.log('Invalid choice. Please try again.');
    }
  }
  
  rl.close();
}

// Handle process termination
process.on('SIGINT', () => {
  console.log('\n\nGoodbye!');
  rl.close();
  process.exit(0);
});

// Start the application
if (require.main === module) {
  main().catch(console.error);
}

module.exports = { OPERATIONS, CONFIG, makeRequest };
