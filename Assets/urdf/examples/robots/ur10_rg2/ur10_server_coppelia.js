// WoT server for UR10+RG2 in CoppeliaSim
// Usage: node /project-root/Assets/urdf/examples/robots/ur10_rg2/ur10_server_coppelia.js
// This implementation communicates with CoppeliaSim via ROS2 using the addOn helper scripts

const { Servient } = require('@node-wot/core');
const HttpServer = require('@node-wot/binding-http').HttpServer;
const rcl = require('rclnodejs');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process'); // For child process management

const { callService } = require('../../../../../library/common/ros2_service_helper');
const { deg2quat } = require('../../../../../library/common/deg2quat');
const PORT = 8084;

// Track child processes (MoveIt stack)
const childProcesses = {
  robot_state_publisher: null,
  joint_state_bridge: null,
  move_group: null
};

// Helper function to convert degrees to radians
function deg2rad(degrees) {
  return degrees * Math.PI / 180;
}

// Helper function to convert radians to degrees
function rad2deg(radians) {
  return radians * 180 / Math.PI;
}

// Global state
let ur10Handle = null;
let currentJointPositions = {
  shoulder_pan_joint: 0,
  shoulder_lift_joint: 0,
  elbow_joint: 0,
  wrist_1_joint: 0,
  wrist_2_joint: 0,
  wrist_3_joint: 0
};

async function main() {
  // Init ROS 2
  await rcl.init();
  const node = new rcl.Node('ur10_server_coppelia');
  let spinInterval = setInterval(() => {
    if (typeof rcl.spinOnce === 'function') rcl.spinOnce(node);
  }, 100);

  // Subscribe to joint states to track current positions
  const jointStateSubscription = node.createSubscription(
    'std_msgs/msg/String',
    '/coppelia/ur10/joint_states_string',
    (msg) => {
      try {
        const data = JSON.parse(msg.data);
        if (data.names && data.positions) {
          const jointOrder = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
          ];
          
          jointOrder.forEach((jointName, index) => {
            const nameIndex = data.names.indexOf(jointName);
            if (nameIndex >= 0 && data.positions[nameIndex] !== undefined) {
              currentJointPositions[jointName] = data.positions[nameIndex];
            }
          });
        }
      } catch (error) {
        console.warn('[JointState] Failed to parse joint state message:', error.message);
      }
    }
  );

  // Get UR10 handle from environment variable or models list
  async function getUR10Handle() {
    if (ur10Handle !== null) {
      return ur10Handle;
    }

    // First, check if handle is provided via environment variable
    const envHandle = process.env.UR10_MODEL_HANDLE;
    if (envHandle) {
      ur10Handle = parseInt(envHandle, 10);
      if (isNaN(ur10Handle)) {
        console.error('[UR10Handle] Invalid handle from environment variable:', envHandle);
        throw new Error('Invalid UR10_MODEL_HANDLE environment variable');
      }
      console.log('[UR10Handle] Using handle from environment variable:', ur10Handle);
      return ur10Handle;
    }

    // Fallback to dynamic discovery from models list
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        reject(new Error('Timeout waiting for models list'));
      }, 5000);

      const subscription = node.createSubscription(
        'std_msgs/msg/String',
        '/coppeliasim/models',
        (msg) => {
          try {
            clearTimeout(timeout);
            node.destroySubscription(subscription);
            
            const models = JSON.parse(msg.data);
            // Look for UR10 model (could be named "UR10", "ur10", or similar)
            const ur10Model = models.find(m => 
              m.name && (m.name.toLowerCase().includes('ur10') || m.name.toLowerCase().includes('ur_10'))
            );
            
            if (ur10Model) {
              ur10Handle = ur10Model.handle;
              console.log('[UR10Handle] Found UR10 handle from models list:', ur10Handle);
              resolve(ur10Handle);
            } else {
              // If not found, use a default handle or the first model
              console.warn('[UR10Handle] UR10 model not found in models list, trying to use first available model');
              if (models.length > 0) {
                ur10Handle = models[0].handle;
                console.log('[UR10Handle] Using first model handle:', ur10Handle);
                resolve(ur10Handle);
              } else {
                reject(new Error('No models found in scene'));
              }
            }
          } catch (error) {
            reject(error);
          }
        }
      );
    });
  }

  // Helper to send UR10 arm command to CoppeliaSim (6 joints only)
  async function sendUR10Command(handle, ur10Positions) {
    const publisher = node.createPublisher('std_msgs/msg/String', '/coppelia/ur10/ur10_joints');
    
    // Ensure positions is a clean array of numbers
    const cleanPositions = ur10Positions.map(pos => Number(pos));
    
    const ur10Json = JSON.stringify({
      handle: handle,
      positions: cleanPositions
    });
    
    publisher.publish({ data: ur10Json });
    console.log('[UR10] Sent command:', ur10Json);
    console.log('[UR10] Positions array:', cleanPositions);
    
    // Wait for movement to complete (simple time-based approach)
    // In a production system, you'd want feedback from CoppeliaSim
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    node.destroyPublisher(publisher);
  }

  // Helper to send gripper command to CoppeliaSim (2 joints only)
  async function sendGripperCommand(handle, gripperPositions) {
    const publisher = node.createPublisher('std_msgs/msg/String', '/coppelia/ur10/gripper');
    
    const gripperJson = JSON.stringify({
      handle: handle,
      positions: gripperPositions
    });
    
    publisher.publish({ data: gripperJson });
    console.log('[Gripper] Sent command:', gripperJson);
    
    // Wait for gripper movement to complete
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    node.destroyPublisher(publisher);
  }

  // WoT setup
  const servient = new Servient();
  servient.addServer(new HttpServer({ port: PORT }));
  const wot = await servient.start();

  // Load Thing Description from external JSON file
  const tdPath = path.join(__dirname, 'ur10_server.json');
  const td = JSON.parse(fs.readFileSync(tdPath, 'utf8'));

  const thing = await wot.produce(td);

  // Action: moveToCartesian
  thing.setActionHandler('moveToCartesian', async (input) => {
    const data = await input.value();
    const pos = data?.position || {};
    const ori = data?.orientation || {};
    
    console.log('[WoT.moveToCartesian] Request:', JSON.stringify(data));

    // Use default values for hidden parameters
    const groupName = 'ur_manipulator';
    const ikLink = 'tool0';
    const frameId = 'base_link';

    // Convert orientation to quaternion
    let orientation;
    const hasQuat = ['x', 'y', 'z', 'w'].every(k => Number.isFinite(ori[k]));
    if (hasQuat) {
      orientation = { x: Number(ori.x), y: Number(ori.y), z: Number(ori.z), w: Number(ori.w) };
    } else {
      const { qx, qy, qz, qw } = deg2quat({
        roll: Number.isFinite(ori.roll) ? ori.roll : 0,
        pitch: Number.isFinite(ori.pitch) ? ori.pitch : 0,
        yaw: Number.isFinite(ori.yaw) ? ori.yaw : 0
      });
      orientation = { x: qx, y: qy, z: qz, w: qw };
    }

    const position = {
      x: Number.isFinite(pos.x) ? pos.x : 0,
      y: Number.isFinite(pos.y) ? pos.y : 0,
      z: Number.isFinite(pos.z) ? pos.z : 0
    };

    // Build IK request for moveit_msgs/srv/GetPositionIK
    const payload = {
      ik_request: {
        group_name: groupName,
        ik_link_name: ikLink,
        avoid_collisions: false,
        pose_stamped: {
          header: { frame_id: frameId },
          pose: { position, orientation }
        },
        robot_state: {
          is_diff: false,
          joint_state: {
            header: { frame_id: frameId },
            name: [],
            position: []
          },
          multi_dof_joint_state: {
            header: { frame_id: frameId },
            joint_names: [],
            transforms: [],
            twist: [],
            wrench: []
          },
          attached_collision_objects: []
        }
      }
    };

    console.log('[WoT.moveToCartesian] Calling IK service...');

    try {
      const { resp } = await callService(
        node,
        {
          srvType: 'moveit_msgs/srv/GetPositionIK',
          serviceName: '/compute_ik',
          payload
        },
        { timeoutMs: 10000, debug: true }
      );

      console.log('[WoT.moveToCartesian] IK response error code:', resp?.error_code?.val);

      // Check IK response error code
      const ikErrorCode = resp?.error_code?.val;
      if (ikErrorCode !== 1) { // 1 = SUCCESS in moveit_msgs/MoveItErrorCodes
        let errorMsg = 'Motion failed: ';
        switch (ikErrorCode) {
          case -31:
            errorMsg += 'No IK solution found for the target pose.';
            break;
          case -32:
            errorMsg += 'IK solution found but violates joint limits.';
            break;
          case -33:
            errorMsg += 'IK solution found but violates path constraints.';
            break;
          default:
            errorMsg += `IK solver failed with error code ${ikErrorCode}.`;
        }
        return errorMsg;
      }

      // Extract joint solution (first 6 joints for UR10)
      const jointPositions = resp?.solution?.joint_state?.position || [];
      
      // Convert to plain array if it's an object with numeric keys
      let ur10Positions;
      if (Array.isArray(jointPositions)) {
        ur10Positions = jointPositions.slice(0, 6);
      } else {
        // Handle case where position might be an object with numeric keys
        ur10Positions = [];
        for (let i = 0; i < 6; i++) {
          const val = jointPositions[i];
          if (typeof val === 'number') {
            ur10Positions.push(val);
          } else if (val !== undefined) {
            ur10Positions.push(Number(val));
          }
        }
      }
      
      if (ur10Positions.length !== 6) {
        console.error('[WoT.moveToCartesian] IK solution incomplete. Got:', ur10Positions);
        return 'Motion failed: IK solution does not contain 6 joint positions.';
      }

      // Ensure all values are valid numbers
      const validPositions = ur10Positions.every(pos => typeof pos === 'number' && !isNaN(pos));
      if (!validPositions) {
        console.error('[WoT.moveToCartesian] Invalid joint positions:', ur10Positions);
        return 'Motion failed: IK solution contains invalid values.';
      }

      console.log('[WoT.moveToCartesian] IK solution (radians):', ur10Positions);
      console.log('[WoT.moveToCartesian] IK solution (degrees):', ur10Positions.map(j => rad2deg(j).toFixed(2)));

      // Get handle and send UR10 command
      const handle = await getUR10Handle();
      
      await sendUR10Command(handle, ur10Positions);

      return 'Motion executed successfully.';
    } catch (error) {
      console.error('[WoT.moveToCartesian] Error:', error);
      return `Motion failed: ${error.message}`;
    }
  });

  // Action: moveToJoint
  thing.setActionHandler('moveToJoint', async (input) => {
    const data = await input.value();
    
    // Validate input object
    if (!data || typeof data !== 'object') {
      return 'Motion failed: Invalid input. Expected object with joint positions.';
    }

    console.log('[WoT.moveToJoint] Request:', JSON.stringify(data));

    const targetJointOrder = [
      'shoulder_pan_joint',
      'shoulder_lift_joint',
      'elbow_joint',
      'wrist_1_joint',
      'wrist_2_joint',
      'wrist_3_joint'
    ];

    try {
      // Get UR10 handle
      const handle = await getUR10Handle();
      
      // Extract positions in the correct order, handle null values, and convert degrees to radians
      const positions = targetJointOrder.map(jointName => {
        const targetValue = data[jointName];
        let finalValue;
        
        if (targetValue === null || targetValue === undefined) {
          // Use current position if target is null/undefined
          if (currentJointPositions[jointName] !== undefined) {
            finalValue = currentJointPositions[jointName];
            console.log(`[WoT.moveToJoint] ${jointName}: using current position ${rad2deg(finalValue).toFixed(2)}° (target was null)`);
          } else {
            throw new Error(`Cannot use current position for ${jointName}: current position not available`);
          }
        } else if (typeof targetValue !== 'number' || !Number.isFinite(targetValue)) {
          throw new Error(`Invalid value for ${jointName}: must be a valid number or null`);
        } else {
          // Convert degrees to radians
          finalValue = deg2rad(targetValue);
          
          console.log(`[WoT.moveToJoint] ${jointName}: target=${targetValue}° (${finalValue.toFixed(6)} rad)`);
        }
        
        return finalValue;
      });

      // Send UR10 command (only arm joints)
      await sendUR10Command(handle, positions);

      return 'Motion executed successfully.';
    } catch (error) {
      console.error('[WoT.moveToJoint] Error:', error);
      return `Motion failed: ${error.message}`;
    }
  });

  // Action: gripOpen
  thing.setActionHandler('gripOpen', async () => {
    try {
      console.log('[WoT.gripOpen] Opening gripper...');
      
      const handle = await getUR10Handle();
      
      // Open gripper: RG2 wide position (0.30 rad for each finger)
      const gripperPositions = [0.30, -0.30];
      
      await sendGripperCommand(handle, gripperPositions);
      
      return { success: true, message: 'Gripper opened' };
    } catch (error) {
      console.error('[WoT.gripOpen] Error:', error);
      return { success: false, message: error.message };
    }
  });

  // Action: gripClose
  thing.setActionHandler('gripClose', async () => {
    try {
      console.log('[WoT.gripClose] Closing gripper...');
      
      const handle = await getUR10Handle();
      
      // Close gripper: RG2 narrow position (0.15 rad for each finger - not too tight)
      const gripperPositions = [0.1, -0.1];
      
      await sendGripperCommand(handle, gripperPositions);
      
      return { success: true, message: 'Gripper closed' };
    } catch (error) {
      console.error('[WoT.gripClose] Error:', error);
      return { success: false, message: error.message };
    }
  });

  // Action: emergencyStop
  thing.setActionHandler('emergencyStop', async () => {
    try {
      console.log('[WoT.emergencyStop] Sending emergency stop command...');
      
      const handle = await getUR10Handle();
      
      // Send current positions to stop movement
      const ur10Positions = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
      ].map(jointName => currentJointPositions[jointName] || 0);
      
      const gripperPositions = [
        currentJointPositions.rg2_finger_joint1 || 0.15,
        currentJointPositions.rg2_finger_joint2 || -0.15
      ];
      
      // Send commands to both UR10 and gripper to stop all movement
      await sendUR10Command(handle, ur10Positions);
      await sendGripperCommand(handle, gripperPositions);
      
      return 'Emergency stop executed successfully.';
    } catch (error) {
      console.error('[WoT.emergencyStop] Error:', error);
      return `Emergency stop failed: ${error.message}`;
    }
  });

  // Property: jointPositions
  thing.setPropertyReadHandler('jointPositions', async () => {
    try {
      const result = {};
      const targetJointOrder = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
      ];

      targetJointOrder.forEach(jointName => {
        if (currentJointPositions[jointName] !== undefined) {
          const positionRad = currentJointPositions[jointName];
          const positionDeg = rad2deg(positionRad);
          result[jointName] = Math.round(positionDeg * 100) / 100; // Round to 0.01 precision
        } else {
          result[jointName] = null;
        }
      });

      return result;
    } catch (error) {
      console.error('[WoT.jointPositions] Error:', error);
      return { error: error.message };
    }
  });

  await thing.expose();
  console.log('UR10Server (CoppeliaSim) exposed on http://localhost:' + PORT + '/ur10_server');
  console.log('');
  console.log('Features:');
  console.log('  ✓ moveToCartesian - Uses MoveIt IK + CoppeliaSim execution');
  console.log('  ✓ moveToJoint - Direct joint control (UR10 arm only)');
  console.log('  ✓ gripOpen/gripClose - RG2 gripper control (independent)');
  console.log('  ✓ emergencyStop - Stop all motion');
  console.log('  ✓ jointPositions - Read current joint positions');
  console.log('');
  console.log('ROS2 Topics:');
  console.log('  → /coppelia/ur10/ur10_joints - UR10 arm commands (6 joints)');
  console.log('  → /coppelia/ur10/gripper - RG2 gripper commands (2 joints)');
  console.log('  ← /coppelia/ur10/joint_states_string - Joint state feedback');
  console.log('');
  console.log('Setup:');
  console.log('1. Load CoppeliaSim scene with UR10+RG2 robot');
  console.log('2. Start the UR10 RG2 Helper in CoppeliaSim console:');
  console.log('   startUR10RG2Helper(<ur10_handle>)');
  console.log('3. Start CoppeliaSim simulation');
  console.log('4. Ensure MoveIt is running for IK (moveToCartesian action)');

  async function shutdown(code = 0) {
    try {
      if (spinInterval) {
        clearInterval(spinInterval);
        spinInterval = null;
      }
      if (jointStateSubscription) {
        node.destroySubscription(jointStateSubscription);
      }
      if (thing) {
        // No explicit unexpose in node-wot API; rely on servient shutdown
      }
      if (servient && typeof servient.shutdown === 'function') {
        await servient.shutdown();
      }
      if (node) {
        node.destroy();
      }
      await rcl.shutdown();
    } catch (e) {
      console.warn('Error during shutdown:', e.message);
    } finally {
      process.exit(code);
    }
  }

  process.on('SIGINT', async () => {
    console.log('\nCaught SIGINT. Shutting down...');
    await shutdown(0);
  });
  process.on('SIGTERM', async () => {
    console.log('\nCaught SIGTERM. Shutting down...');
    await shutdown(0);
  });
}

main().catch((e) => {
  console.error('Failed to start WoT UR10 CoppeliaSim controller:', e);
  process.exit(1);
});

