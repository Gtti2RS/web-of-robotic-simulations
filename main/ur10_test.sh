#!/bin/bash

# UR10 Controller Test Sequence
# This script performs a pick and place operation using WoT actions

BASE_URL="http://localhost:8083/ur10_controller"

echo "Starting UR10 test sequence..."

# Function to send WoT action request
send_action() {
    local action_name="$1"
    local input_data="$2"
    local description="$3"
    
    echo "Step: $description"
    echo "Sending $action_name with input: $input_data"
    
    response=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "$input_data" \
        "$BASE_URL/actions/$action_name")
    
    echo "Response: $response"
    echo "---"
}

# Step 1: Move to initial position
echo "=== Step 1: Move to initial position ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": 0.5, "z": 0.3 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move to pick position"

# Step 2: Open gripper
echo "=== Step 2: Open gripper ==="
send_action "gripOpen" '{}' "Open gripper to prepare for pickup"

# Step 3: Move down to z:0.22
echo "=== Step 3: Move down to z:0.22 ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": 0.5, "z": 0.22 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move down to pickup height"

# Step 4: Close gripper
echo "=== Step 4: Close gripper ==="
send_action "gripClose" '{}' "Close gripper to grasp object"

# Step 5: Move to place position
echo "=== Step 5: Move to place position ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": -0.5, "z": 0.25 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move to place position"

# Step 6: Open gripper
echo "=== Step 6: Open gripper ==="
send_action "gripOpen" '{}' "Open gripper to release object"

# Step 7: Move to z:0.22
echo "=== Step 7: Move to z:0.22 ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": -0.5, "z": 0.22 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move down to pickup height"

# Step 8: Close gripper
echo "=== Step 8: Close gripper ==="
send_action "gripClose" '{}' "Close gripper to grasp object"

# Step 9: Move to y:0.5, z:0.25
echo "=== Step 9: Move to y:0.5, z:0.25 ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": 0.5, "z": 0.25 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move to place position"

# Step 10: Open gripper
echo "=== Step 10: Open gripper ==="
send_action "gripOpen" '{}' "Open gripper to release object"

echo "=== Test sequence completed ==="
