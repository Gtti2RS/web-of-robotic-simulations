// Minimal WoT server exposing an IK action
// Usage: node /home/yifan/wos/bot_servers/ur10_controller.js

const { Servient } = require('@node-wot/core');
const HttpServer = require('@node-wot/binding-http').HttpServer;
const rcl = require('rclnodejs');
const fs = require('fs');
const path = require('path');

const { callService, callAction } = require('../library/common/ros2_service_helper');
const { deg2quat } = require('../library/common/deg2quat');
const PORT = 8083;

// Helper function to convert degrees to radians
function deg2rad(degrees) {
  return degrees * Math.PI / 180;
}

// Helper function to normalize angle to find nearest equivalent position
function normalizeAngle(degrees) {
  while (degrees > 180) degrees -= 360;
  while (degrees < -180) degrees += 360;
  return degrees;
}

// Helper function to find nearest equivalent position for continuous joints
function findNearestEquivalent(targetRadians, currentRadians) {
  // Convert to degrees for easier calculation
  const targetDegrees = targetRadians * 180 / Math.PI;
  const currentDegrees = currentRadians * 180 / Math.PI;
  
  // Try the target as-is
  let bestTargetDegrees = targetDegrees;
  let minDiff = Math.abs(targetDegrees - currentDegrees);
  
  // Try target - 360° (one full rotation back)
  const targetMinus360 = targetDegrees - 360;
  const diffMinus360 = Math.abs(targetMinus360 - currentDegrees);
  if (diffMinus360 < minDiff) {
    bestTargetDegrees = targetMinus360;
    minDiff = diffMinus360;
  }
  
  // Try target + 360° (one full rotation forward)
  const targetPlus360 = targetDegrees + 360;
  const diffPlus360 = Math.abs(targetPlus360 - currentDegrees);
  if (diffPlus360 < minDiff) {
    bestTargetDegrees = targetPlus360;
    minDiff = diffPlus360;
  }
  
  // Convert back to radians
  const bestTargetRadians = bestTargetDegrees * Math.PI / 180;
  
  console.log(`[findNearestEquivalent] target=${targetDegrees.toFixed(2)}°, current=${currentDegrees.toFixed(2)}°, best=${bestTargetDegrees.toFixed(2)}°, diff=${minDiff.toFixed(2)}° (${bestTargetRadians.toFixed(6)} rad)`);
  return bestTargetRadians;
}


// Helper function to check trajectory execution result and return appropriate message
function checkTrajectoryResult(result) {
  const trajectoryErrorCode = result?.error_code;
  if (trajectoryErrorCode !== 0) { // 0 = SUCCESS in control_msgs/FollowJointTrajectory
    let errorMsg = 'Motion failed: ';
    switch (trajectoryErrorCode) {
      case -1:
        errorMsg += 'Invalid trajectory - start point deviates from current robot state.';
        break;
      case -2:
        errorMsg += 'Invalid goal - trajectory goal is invalid.';
        break;
      case -3:
        errorMsg += 'Path tolerance violated during execution.';
        break;
      case -4:
        errorMsg += 'Goal tolerance violated during execution.';
        break;
      default:
        errorMsg += `Trajectory execution failed with error code ${trajectoryErrorCode}.`;
    }
    return errorMsg;
  }
  return 'Motion executed successfully.';
}


async function main() {
  // Init ROS 2
  await rcl.init();
  const node = new rcl.Node('ur10_controller');
  let spinInterval = setInterval(() => {
    if (typeof rcl.spinOnce === 'function') rcl.spinOnce(node);
  }, 100);

  // WoT setup
  const servient = new Servient();
  servient.addServer(new HttpServer({ port: PORT }));
  const wot = await servient.start();

  // Load Thing Description from external JSON file
  const tdPath = path.join(__dirname, 'ur10_controller.json');
  const td = JSON.parse(fs.readFileSync(tdPath, 'utf8'));

  const thing = await wot.produce(td);

  thing.setActionHandler('moveToCartesian', async (input) => {
    const data = await input.value();
    const pos = data?.position || {};
    const ori = data?.orientation || {};
    // Use default values for hidden parameters
    const groupName = 'ur_manipulator';
    const ikLink = 'tool0';
    const frameId = 'world';

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

    

    // Build IK request for moveit_msgs/srv/GetPositionIK with safe seed
    const targetJointOrder = [
      'shoulder_pan_joint',
      'shoulder_lift_joint',
      'elbow_joint',
      'wrist_1_joint',
      'wrist_2_joint',
      'wrist_3_joint'
    ];
    const safePositions = [
      2.227278196386671, -0.8463464839324557, 1.7548443268360512,
      2.233094810147053, -3.7980745231639905, -4.712388981224473
    ];
    
    const payload = {
      ik_request: {
        group_name: groupName,
        ik_link_name: ikLink,
        avoid_collisions: false, // currently collision avoidance is not working (doesn't return valid solution), maybe due to wrong collision models
        pose_stamped: {
          header: { frame_id: frameId },
          pose: { position, orientation }
        },
        robot_state: {
          joint_state: {
            header: { frame_id: frameId },
            name: targetJointOrder,
            position: safePositions
          },
          multi_dof_joint_state: {
            header: { frame_id: frameId },
            joint_names: [],
            transforms: [],
            twist: [],
            wrench: []
          },
          attached_collision_objects: [],
          is_diff: false
        }
      }
    };

    console.log('[WoT.moveToCartesian] Request:', JSON.stringify(payload));

    const { resp } = await callService(
      node,
      {
        srvType: 'moveit_msgs/srv/GetPositionIK',
        serviceName: '/compute_ik',
        payload
      },
      { timeoutMs: 10000, debug: true }
    );

    console.log('[WoT.moveToCartesian] IK response:', JSON.stringify(resp));

    // Check IK response error code first
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

    // Extract joint solution and send to FollowJointTrajectory action
    const names = resp?.solution?.joint_state?.name || [];
    const positionsData = resp?.solution?.joint_state?.position || [];

    function getPosAt(index) {
      if (Array.isArray(positionsData)) return positionsData[index];
      const key = String(index);
      return positionsData[key];
    }

    const jointPositions = targetJointOrder.map(j => {
      const idx = names.indexOf(j);
      if (idx < 0) return undefined;
      const val = getPosAt(idx);
      return typeof val === 'number' ? val : Number(val);
    });

    // Get current joint positions to find nearest equivalent positions
    let currentPositionsForNormalization = {};
    try {
      const jointStateTopic = '/joint_states';
      const jointStateMsg = await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => {
          reject(new Error('Timeout waiting for current joint state'));
        }, 2000);

        const subscription = node.createSubscription(
          'sensor_msgs/msg/JointState',
          jointStateTopic,
          (msg) => {
            clearTimeout(timeout);
            node.destroySubscription(subscription);
            resolve(msg);
          }
        );
      });

      targetJointOrder.forEach(jointName => {
        const index = jointStateMsg.name.indexOf(jointName);
        if (index >= 0) {
          const positionRad = jointStateMsg.position[index];
          currentPositionsForNormalization[jointName] = positionRad * 180 / Math.PI; // Convert to degrees
        }
      });
    } catch (error) {
      console.warn('[WoT.moveToCartesian] Could not get current positions for normalization:', error.message);
    }

    // Process joint positions to find nearest equivalent positions for continuous joints
    const processedJointPositions = targetJointOrder.map((jointName, index) => {
      const originalPosition = jointPositions[index];
      if (originalPosition === undefined) return originalPosition;
      
      // Only process continuous joints (shoulder_pan_joint and wrist_3_joint)
      if (jointName === 'shoulder_pan_joint' || jointName === 'wrist_3_joint') {
        if (currentPositionsForNormalization[jointName] !== undefined) {
          const currentRadians = currentPositionsForNormalization[jointName] * Math.PI / 180; // Convert current position to radians
          const nearestEquivalentRadians = findNearestEquivalent(originalPosition, currentRadians);
          
          console.log(`[WoT.moveToCartesian] ${jointName} normalization: original=${(originalPosition * 180 / Math.PI).toFixed(2)}°, current=${currentPositionsForNormalization[jointName].toFixed(2)}°, nearest=${(nearestEquivalentRadians * 180 / Math.PI).toFixed(2)}° (${nearestEquivalentRadians.toFixed(6)} rad)`);
          return nearestEquivalentRadians;
        }
      }
      
      return originalPosition;
    });


    // Get current joint positions for splitting logic
    let currentPositions = {};
    try {
      const jointStateTopic = '/joint_states';
      const jointStateMsg = await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => {
          reject(new Error('Timeout waiting for current joint state'));
        }, 2000);

        const subscription = node.createSubscription(
          'sensor_msgs/msg/JointState',
          jointStateTopic,
          (msg) => {
            clearTimeout(timeout);
            node.destroySubscription(subscription);
            resolve(msg);
          }
        );
      });

      targetJointOrder.forEach(jointName => {
        const index = jointStateMsg.name.indexOf(jointName);
        if (index >= 0) {
          const positionRad = jointStateMsg.position[index];
          currentPositions[jointName] = positionRad * 180 / Math.PI; // Convert to degrees
          console.log(`[WoT.moveToCartesian] Current ${jointName}: ${currentPositions[jointName].toFixed(2)}°`);
        }
      });
    } catch (error) {
      console.warn('[WoT.moveToCartesian] Could not get current positions, using single movement:', error.message);
    }

    // Execute movement (with automatic splitting if needed)
    return await executeSplitMovements(node, targetJointOrder, currentPositions, processedJointPositions, processedJointPositions, 'cartesian');
  });

  thing.setActionHandler('moveToJoint', async (input) => {
    const data = await input.value();
    
    // Validate input object
    if (!data || typeof data !== 'object') {
      return 'Motion failed: Invalid input. Expected object with joint positions.';
    }

    const targetJointOrder = [
      'shoulder_pan_joint',
      'shoulder_lift_joint',
      'elbow_joint',
      'wrist_1_joint',
      'wrist_2_joint',
      'wrist_3_joint'
    ];

    // Get current joint positions to calculate shortest path
    let currentPositions = {};
    try {
      const jointStateTopic = '/joint_states';
      const jointStateMsg = await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => {
          reject(new Error('Timeout waiting for current joint state'));
        }, 2000);

        const subscription = node.createSubscription(
          'sensor_msgs/msg/JointState',
          jointStateTopic,
          (msg) => {
            clearTimeout(timeout);
            node.destroySubscription(subscription);
            resolve(msg);
          }
        );
      });

      targetJointOrder.forEach(jointName => {
        const index = jointStateMsg.name.indexOf(jointName);
        if (index >= 0) {
          const positionRad = jointStateMsg.position[index];
          currentPositions[jointName] = positionRad * 180 / Math.PI; // Convert to degrees
          console.log(`[WoT.moveToJoint] Current ${jointName}: ${currentPositions[jointName].toFixed(2)}°`);
        }
      });
    } catch (error) {
      console.warn('[WoT.moveToJoint] Could not get current positions, using target positions directly:', error.message);
    }

    // Extract positions in the correct order, handle null values, and convert degrees to radians
    let positions;
    try {
      positions = targetJointOrder.map(jointName => {
        const targetValue = data[jointName];
        let finalValue;
        
        if (targetValue === null || targetValue === undefined) {
          // Use current position if target is null/undefined
          if (currentPositions[jointName] !== undefined) {
            finalValue = currentPositions[jointName];
            console.log(`[WoT.moveToJoint] ${jointName}: using current position ${finalValue.toFixed(2)}° (target was null)`);
          } else {
            throw new Error(`Cannot use current position for ${jointName}: current position not available`);
          }
        } else if (typeof targetValue !== 'number' || !Number.isFinite(targetValue)) {
          throw new Error(`Invalid value for ${jointName}: must be a valid number or null`);
        } else {
          // Use target value directly
          finalValue = targetValue;
          
          // Log the values for debugging
          if (currentPositions[jointName] !== undefined) {
            const currentValue = currentPositions[jointName];
            console.log(`[WoT.moveToJoint] ${jointName}: current=${currentValue.toFixed(2)}°, target=${targetValue}°, sending=${finalValue.toFixed(2)}°`);
          }
        }
        
        return deg2rad(finalValue); // Convert degrees to radians
      });
    } catch (error) {
      return `Motion failed: ${error.message}`;
    }

        // Execute movement (with automatic splitting if needed)
        return await executeSplitMovements(node, targetJointOrder, currentPositions, data, positions, 'joint');
  });

  // Emergency stop action
  thing.setActionHandler('emergencyStop', async () => {
    try {
      console.log('[WoT.emergencyStop] Sending emergency stop command...');
      
      // Method 1: First try to cancel any ongoing goal
      try {
        console.log('[WoT.emergencyStop] Attempting to cancel ongoing goal...');
        
        await callAction(
          node,
          {
            actionType: 'control_msgs/action/FollowJointTrajectory',
            actionName: '/ur_arm_controller/follow_joint_trajectory',
            goal: {},
            cancel: true
          },
          { timeoutMs: 1000, collectFeedback: false }
        );
        console.log('[WoT.emergencyStop] Goal cancel command sent');
        
        // Wait a bit for the cancel to take effect
        await new Promise(resolve => setTimeout(resolve, 100));
        
      } catch (cancelError) {
        console.warn('[WoT.emergencyStop] Cancel command failed:', cancelError.message);
      }
      
      // Method 2: Send zero velocity commands to stop movement
      try {
        console.log('[WoT.emergencyStop] Attempting to send zero velocity commands...');
        
        const targetJointOrder = [
          'shoulder_pan_joint',
          'shoulder_lift_joint',
          'elbow_joint',
          'wrist_1_joint',
          'wrist_2_joint',
          'wrist_3_joint'
        ];

        // Send zero velocity commands
        const zeroVelocityMsg = {
          name: targetJointOrder,
          velocity: [0, 0, 0, 0, 0, 0],
          effort: [0, 0, 0, 0, 0, 0]
        };

        // Try to publish to joint command topic
        const publisher = node.createPublisher('sensor_msgs/msg/JointState', '/ur_arm_controller/joint_command');
        publisher.publish(zeroVelocityMsg);
        
        // Also try the trajectory controller's command topic
        const trajectoryPublisher = node.createPublisher('trajectory_msgs/msg/JointTrajectory', '/ur_arm_controller/joint_trajectory');
        const stopTrajectory = {
          joint_names: targetJointOrder,
          points: [
            {
              positions: [0, 0, 0, 0, 0, 0],
              velocities: [0, 0, 0, 0, 0, 0],
              time_from_start: { sec: 0, nanosec: 100000000 } // 0.1 seconds
            }
          ]
        };
        trajectoryPublisher.publish(stopTrajectory);

        console.log('[WoT.emergencyStop] Zero velocity commands sent');
        return 'Emergency stop executed successfully.';
        
      } catch (error) {
        console.warn('[WoT.emergencyStop] Zero velocity method failed:', error.message);
        return 'Emergency stop attempted (zero velocity method failed).';
      }
    } catch (error) {
      console.error('[WoT.emergencyStop] Error:', error);
      return `Emergency stop failed: ${error.message}`;
    }
  });

  // Property handler for current joint positions
  thing.setPropertyReadHandler('jointPositions', async () => {
    try {
      // Subscribe to joint state topic to get current positions
      const jointStateTopic = '/joint_states';
      const jointStateMsg = await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => {
          reject(new Error('Timeout waiting for joint state'));
        }, 5000);

        const subscription = node.createSubscription(
          'sensor_msgs/msg/JointState',
          jointStateTopic,
          (msg) => {
            clearTimeout(timeout);
            node.destroySubscription(subscription);
            resolve(msg);
          }
        );
      });

      const targetJointOrder = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
      ];

      const result = {};
      targetJointOrder.forEach(jointName => {
        const index = jointStateMsg.name.indexOf(jointName);
        if (index >= 0) {
          const positionRad = jointStateMsg.position[index];
          const positionDeg = positionRad * 180 / Math.PI; // Convert to degrees
          result[jointName] = Math.round(positionDeg * 100) / 100; // Round to 0.01 precision
        } else {
          result[jointName] = null;
        }
      });

      return result;
    } catch (error) {
      console.error('[WoT.currentJointPositions] Error:', error);
      return { error: error.message };
    }
  });

  // RG2 gripper actions via FollowJointTrajectory
  const gripperJoints = ['rg2_finger_joint1','rg2_finger_joint2'];
  const gripperAction = '/rg2_trajectory_controller/follow_joint_trajectory';

  thing.setActionHandler('gripOpen', async () => {
    const goal = {
      trajectory: {
        joint_names: gripperJoints,
        points: [
          { positions: [0.30, 0.30], time_from_start: { sec: 1, nanosec: 0 } }
        ]
      }
    };
    const { result } = await callAction(
      node,
      { actionType: 'control_msgs/action/FollowJointTrajectory', actionName: gripperAction, goal },
      { timeoutMs: 20000, collectFeedback: false }
    );
    console.log('[WoT.gripOpen] result:', JSON.stringify(result));
    return result;
  });

  thing.setActionHandler('gripClose', async () => {
    const goal = {
      trajectory: {
        joint_names: gripperJoints,
        points: [
          { positions: [0.10, 0.10], time_from_start: { sec: 1, nanosec: 0 } }
        ]
      }
    };
    const { result } = await callAction(
      node,
      { actionType: 'control_msgs/action/FollowJointTrajectory', actionName: gripperAction, goal },
      { timeoutMs: 20000, collectFeedback: false }
    );
    console.log('[WoT.gripClose] result:', JSON.stringify(result));
    return result;
  });

  await thing.expose();
  console.log('UR10Controller exposed on http://localhost:' + PORT + '/ur10_controller');

  async function shutdown(code = 0) {
    try {
      if (spinInterval) {
        clearInterval(spinInterval);
        spinInterval = null;
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

// Helper function to execute split movements for shoulder_pan_joint
// For the shoulder_pan_joint, the controller can't find the shortest path for movements >120°
// so I split the movement into smaller movements of 90° as workaround.
// This function can handle both joint movements (from moveToJoint) and Cartesian movements (from moveToCartesian)
async function executeSplitMovements(node, targetJointOrder, currentPositions, targetPositions, positions, movementType = 'joint') {
  // Check if shoulder_pan_joint needs splitting (for both increasing and decreasing movements >90°)
  const needsSplitting = targetJointOrder.some((jointName, index) => {
    if (jointName === 'shoulder_pan_joint' && currentPositions[jointName] !== undefined) {
      const currentValue = currentPositions[jointName];
      let targetValue;
      
      if (movementType === 'joint') {
        // For joint movements, targetPositions is the data object with joint values
        targetValue = targetPositions[jointName];
        
        // Skip splitting check if target is null (using current position)
        if (targetValue === null || targetValue === undefined) {
          console.log(`[WoT.moveToJoint] ${jointName} check: using current position, no splitting needed`);
          return false;
        }
      } else {
        // For Cartesian movements, targetPositions is the array of joint positions from IK (already normalized)
        targetValue = targetPositions[index] * 180 / Math.PI; // Convert from radians to degrees
        console.log(`[WoT.moveToCartesian] ${jointName} splitting check: targetPositions[${index}]=${targetPositions[index].toFixed(6)} rad = ${targetValue.toFixed(2)}°`);
      }
      
      const diff = Math.abs(targetValue - currentValue);
      
      // Split if: absolute difference > 90° (both increasing and decreasing)
      console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] ${jointName} check: current=${currentValue.toFixed(2)}°, target=${targetValue.toFixed(2)}°, diff=${diff.toFixed(2)}°, needsSplit=${diff > 90}`);
      return diff > 90;
    }
    return false;
  });

  if (!needsSplitting) {
    console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] No splitting needed, executing single movement...`);
    // Execute single movement
    const goal = {
      trajectory: {
        joint_names: targetJointOrder,
        points: [
          {
            positions: positions,
            time_from_start: { sec: 1, nanosec: 0 }
          }
        ]
      }
    };

    console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] Sending single trajectory:`, JSON.stringify(goal));

    const { result } = await callAction(
      node,
      {
        actionType: 'control_msgs/action/FollowJointTrajectory',
        actionName: '/ur_arm_controller/follow_joint_trajectory',
        goal
      },
      { timeoutMs: 60000, collectFeedback: false }
    );

    console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] Trajectory result:`, JSON.stringify(result));
    
    // Check trajectory execution result
    return checkTrajectoryResult(result);
  }

  console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] Large movement detected for shoulder_pan_joint, splitting into up to 4 requests...`);

  // Find the maximum difference for shoulder_pan_joint to determine number of splits
  let maxDiff = 0;
  if (currentPositions['shoulder_pan_joint'] !== undefined) {
    const currentValue = currentPositions['shoulder_pan_joint'];
    let targetValue;
    
    if (movementType === 'joint') {
      targetValue = targetPositions['shoulder_pan_joint'];
      
      // Skip if target is null (using current position)
      if (targetValue !== null && targetValue !== undefined) {
        const diff = Math.abs(targetValue - currentValue);
        maxDiff = diff;
        console.log(`[WoT.moveToJoint] shoulder_pan_joint splitting: current=${currentValue.toFixed(2)}°, target=${targetValue.toFixed(2)}°, abs_diff=${diff.toFixed(2)}°`);
      } else {
        console.log(`[WoT.moveToJoint] shoulder_pan_joint splitting: using current position, no splitting needed`);
      }
    } else {
      // For Cartesian movements, get the shoulder_pan_joint index
      const shoulderPanIndex = targetJointOrder.indexOf('shoulder_pan_joint');
      targetValue = targetPositions[shoulderPanIndex] * 180 / Math.PI; // Convert from radians to degrees
      const diff = Math.abs(targetValue - currentValue);
      maxDiff = diff;
      console.log(`[WoT.moveToCartesian] shoulder_pan_joint splitting: current=${currentValue.toFixed(2)}°, target=${targetValue.toFixed(2)}°, abs_diff=${diff.toFixed(2)}° (targetPositions[${shoulderPanIndex}]=${targetPositions[shoulderPanIndex].toFixed(6)} rad)`);
    }
  }

  // Calculate number of splits prioritizing 90° steps
  const num90Steps = Math.floor(maxDiff / 90); // Number of 90° steps
  const remainder = maxDiff % 90; // Remaining degrees for last step
  const actualNumSplits = remainder > 0 ? num90Steps + 1 : num90Steps; // Add 1 if there's a remainder
  const cappedSplits = Math.min(4, actualNumSplits); // Cap at 4 splits
  
  console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] Splitting into ${cappedSplits} movements (max diff: ${maxDiff.toFixed(2)}°, ${num90Steps} x 90° steps + ${remainder.toFixed(2)}° remainder)`);

  let currentPositionsCopy = { ...currentPositions };
  let lastResult = 'Motion executed successfully.';

  // Execute each split movement
  for (let i = 0; i < cappedSplits; i++) {
    const isLastSplit = (i === cappedSplits - 1);
    
    // Calculate target positions for this split
    const splitPositions = targetJointOrder.map((jointName, index) => {
      if (jointName === 'shoulder_pan_joint' && currentPositionsCopy[jointName] !== undefined) {
        const currentValue = currentPositionsCopy[jointName];
        let targetValue;
        
        if (movementType === 'joint') {
          targetValue = targetPositions[jointName];
          
          // If target is null, use current position
          if (targetValue === null || targetValue === undefined) {
            return currentValue;
          }
        } else {
          // For Cartesian movements, targetPositions is the array of joint positions from IK (already normalized)
          targetValue = targetPositions[index] * 180 / Math.PI; // Convert from radians to degrees
        }
        
        const diff = targetValue - currentValue;
        const absDiff = Math.abs(diff);
        
        if (absDiff > 90) {
          let newPosition;
          let stepSize;
          
          if (isLastSplit) {
            // Last step: go directly to target
            newPosition = targetValue;
            stepSize = newPosition - currentValue;
          } else {
            // Regular steps: move by 90° (or remaining 90° steps)
            const direction = diff > 0 ? 1 : -1; // Positive or negative direction
            const stepsRemaining = cappedSplits - i;
            const remainingDistance = targetValue - currentValue;
            
            if (stepsRemaining === 1) {
              // This is the last step, go to target
              newPosition = targetValue;
            } else {
              // Move by 90° in the correct direction
              newPosition = currentValue + (90 * direction);
            }
            stepSize = newPosition - currentValue;
          }
          
          const stepDiff = Math.abs(stepSize);
          console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] ${jointName} split ${i + 1}: ${currentValue.toFixed(2)}° → ${newPosition.toFixed(2)}° (step: ${stepSize.toFixed(2)}°, step_diff: ${stepDiff.toFixed(2)}°)`);
          
          // Convert back to radians for Cartesian movements
          if (movementType === 'cartesian') {
            const resultRadians = newPosition * Math.PI / 180;
            console.log(`[WoT.moveToCartesian] ${jointName} split ${i + 1} conversion: ${newPosition.toFixed(2)}° → ${resultRadians.toFixed(6)} rad`);
            return resultRadians;
          } else {
            return newPosition;
          }
        } else {
          // No splitting needed for this joint
          if (movementType === 'cartesian') {
            // targetValue is in degrees, convert to radians
            const resultRadians = targetValue * Math.PI / 180;
            console.log(`[WoT.moveToCartesian] ${jointName} no split conversion: ${targetValue.toFixed(2)}° → ${resultRadians.toFixed(6)} rad`);
            return resultRadians;
          } else {
            return targetValue; // Joint movements are already in degrees
          }
        }
      } else {
        // For all other joints, use the original target position
        if (movementType === 'joint') {
          const targetValue = targetPositions[jointName];
          if (targetValue === null || targetValue === undefined) {
            return currentPositionsCopy[jointName] || positions[index];
          }
          return targetValue;
        } else {
          // For Cartesian movements, use the original joint positions from IK (already in radians)
          return targetPositions[index];
        }
      }
    });

    const splitGoal = {
      trajectory: {
        joint_names: targetJointOrder,
        points: [
          {
            positions: movementType === 'joint' ? splitPositions.map(deg => deg2rad(deg)) : splitPositions,
            time_from_start: { sec: 1, nanosec: 0 }
          }
        ]
      }
    };

    console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] Split ${i + 1}/${cappedSplits} trajectory:`, JSON.stringify(splitGoal));

    const { result: splitResult } = await callAction(
      node,
      {
        actionType: 'control_msgs/action/FollowJointTrajectory',
        actionName: '/ur_arm_controller/follow_joint_trajectory',
        goal: splitGoal
      },
      { timeoutMs: 60000, collectFeedback: false }
    );

    console.log(`[WoT.moveTo${movementType === 'joint' ? 'Joint' : 'Cartesian'}] Split ${i + 1}/${cappedSplits} result:`, JSON.stringify(splitResult));
    
    // Check result
    lastResult = checkTrajectoryResult(splitResult);
    if (lastResult !== 'Motion executed successfully.') {
      return lastResult;
    }

    // Update current positions for next iteration
    targetJointOrder.forEach((jointName, index) => {
      if (currentPositionsCopy[jointName] !== undefined) {
        if (movementType === 'joint') {
          // For joint movements, splitPositions are in degrees
          currentPositionsCopy[jointName] = splitPositions[index];
        } else {
          // For Cartesian movements, splitPositions are in radians, convert to degrees for tracking
          currentPositionsCopy[jointName] = splitPositions[index] * 180 / Math.PI;
        }
      }
    });

    // Wait between movements (except after the last one)
    if (!isLastSplit) {
      await new Promise(resolve => setTimeout(resolve, 500));
    }
  }

  return lastResult;
}

main().catch((e) => {
  console.error('Failed to start WoT UR10 controller:', e);
  process.exit(1);
});


