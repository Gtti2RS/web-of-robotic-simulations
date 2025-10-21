#!/bin/bash
# Script to start MoveIt move_group for UR10+RG2

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Trap to handle cleanup on exit
cleanup() {
    echo "Shutting down MoveIt stack..."
    # Kill all child processes
    pkill -P $$
    exit 0
}

trap cleanup SIGTERM SIGINT

# Check if move_group is already running
if pgrep -f "move_group.*ur10_rg2_moveit_params" > /dev/null; then
    echo "MoveIt move_group is already running!"
    echo "PID: $(pgrep -f 'move_group.*ur10_rg2_moveit_params')"
    exit 0
fi

echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat /project-root/Assets/urdf/examples/robots/ur10_rg2/ur10_rg2_coppelia.urdf)" \
  > /tmp/robot_state_publisher.log 2>&1 &

echo "Starting joint_state_bridge (converts CoppeliaSim data to JointState)..."
python3 /project-root/library/coppeliasim/joint_state_bridge.py > /tmp/joint_state_bridge.log 2>&1 &

sleep 2

echo "Starting MoveIt move_group..."

# Start move_group in the background
ros2 run moveit_ros_move_group move_group \
  --ros-args \
  --params-file /project-root/Assets/urdf/examples/robots/ur10_rg2/ur10_rg2_moveit_params.yaml \
  > /tmp/move_group.log 2>&1 &

# Wait a moment for it to start
sleep 2

# Check if it started successfully
if pgrep -f "move_group.*ur10_rg2_moveit_params" > /dev/null; then
    echo "✓ MoveIt move_group started successfully!"
    echo "PID: $(pgrep -f 'move_group.*ur10_rg2_moveit_params')"
    echo ""
    echo "Services available:"
    echo "  - /compute_ik (inverse kinematics)"
    echo "  - /plan_kinematic_path (motion planning)"
    echo "  - /compute_fk (forward kinematics)"
    echo ""
    echo "Log file: /tmp/move_group.log"
    
    # Keep the script running to maintain the process group
    # Wait for all background jobs
    echo ""
    echo "MoveIt stack is running. Keeping script alive..."
    wait
else
    echo "✗ Failed to start move_group. Check logs:"
    echo "  tail -50 /tmp/move_group.log"
    exit 1
fi

