#!/bin/bash
set -e

# Fix line endings for Windows Docker compatibility
echo "Fixing line endings for Windows Docker compatibility..."
find /project-root -name "*.sh" -type f -exec sed -i 's/\r$//' {} \; 2>/dev/null || true

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Always ensure baked-in packages are available in project root
# This handles both initial startup and manual exec scenarios

# Copy baked-in packages to project root (since volume mount overwrites /project-root)
if [ -d "/opt/ros_gz_bridge_addon" ] && [ ! -d "/project-root/ros_gz_bridge_addon" ]; then
    echo "Copying baked-in ROS2 workspace to project root..."
    cp -r /opt/ros_gz_bridge_addon /project-root/
    echo "Copied ROS2 workspace from /opt/ros_gz_bridge_addon"
fi

# Copy baked-in npm modules to project root (since volume mount overwrites /project-root)
if [ -d "/opt/node_modules" ] && [ ! -f "/project-root/node_modules/rclnodejs/package.json" ]; then
    echo "Copying baked-in npm modules to project root..."
    # Remove empty directory created by volume mount and copy the real one
    rm -rf /project-root/node_modules
    cp -r /opt/node_modules /project-root/
    echo "Copied npm modules from /opt/node_modules"
fi

# Source baked-in workspace
if [ -f "/opt/ros_gz_bridge_addon/install/setup.bash" ]; then
    source /opt/ros_gz_bridge_addon/install/setup.bash
    echo "Sourced ROS2 workspace from baked-in packages: /opt/ros_gz_bridge_addon/install/setup.bash"
else
    echo "Baked-in ROS2 workspace not found at /opt/ros_gz_bridge_addon/install/setup.bash"
fi

# Wait for other services to be ready
echo "Waiting for simulators to be ready..."
sleep 10

# Change to main directory
cd /project-root/main

# Start file uploader
echo "Starting file uploader..."
node fileUploader.js &

# Start only gz_controller in background
echo "Starting gz_controller..."
node gz_controller.js &
# Wait for gz_controller process
wait

