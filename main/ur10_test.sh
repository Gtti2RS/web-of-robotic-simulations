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
    "position": { "x": -0.5, "y": 0.5, "z": 0.05 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move to pick position"

# Step 2: Open gripper
echo "=== Step 2: Open gripper ==="
send_action "gripOpen" '{}' "Open gripper to prepare for pickup"

# Step 3: Move down to z:0.22
echo "=== Step 3: Move down to z:0.22 ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": 0.5, "z": 0.01 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move down to pickup height"

# Step 4: Close gripper
echo "=== Step 4: Close gripper ==="
send_action "gripClose" '{}' "Close gripper to grasp object"

# Step 5: Move to z:0.05
echo "=== Step 5: Move to z:0.05 ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": 0.5, "z": 0.05 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move to pickup height"

# Step 6: Move to place position
echo "=== Step 6: Move to place position ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": -0.5, "z": 0.02 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move to place position"

# Step 7: Open gripper
echo "=== Step 7: Open gripper ==="
send_action "gripOpen" '{}' "Open gripper to release object"

# Step 8: Move to z:0.01
echo "=== Step 8: Move to z:0.01 ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": -0.5, "z": 0.01 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move down to pickup height"

# Step 9: Close gripper
echo "=== Step 9: Close gripper ==="
send_action "gripClose" '{}' "Close gripper to grasp object"

# Step 10: Move to z:0.02
echo "=== Step 10: Move to z:0.02 ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": -0.5, "z": 0.02 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move to place height"

# Step 11: Move to place position
echo "=== Step 11: Move to place position ==="
send_action "moveToCartesian" '{
    "position": { "x": -0.5, "y": 0.5, "z": 0.02 },
    "orientation": { "roll": 0, "pitch": 90, "yaw": 0 }
}' "Move to place position"


# Step 12: Open gripper
echo "=== Step 12: Open gripper ==="
send_action "gripOpen" '{}' "Open gripper to release object"

echo "=== Test sequence completed ==="
