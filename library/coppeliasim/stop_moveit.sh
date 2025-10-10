#!/bin/bash
# Script to stop MoveIt move_group

echo "Stopping MoveIt move_group..."

# Kill move_group process
pkill -f "move_group.*ur10_rg2_moveit_params"

# Kill TF publishers and bridge
pkill -f "robot_state_publisher"
pkill -f "joint_state_bridge.py"

# Wait a moment
sleep 1

# Check if it's stopped
if pgrep -f "move_group.*ur10_rg2_moveit_params" > /dev/null; then
    echo "✗ Failed to stop move_group. Trying force kill..."
    pkill -9 -f "move_group.*ur10_rg2_moveit_params"
    sleep 1
    
    if pgrep -f "move_group.*ur10_rg2_moveit_params" > /dev/null; then
        echo "✗ Still running!"
        exit 1
    else
        echo "✓ MoveIt move_group stopped (force killed)"
    fi
else
    echo "✓ MoveIt move_group stopped successfully"
fi

