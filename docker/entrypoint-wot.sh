#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
if [ -f '/project-root/ros_gz_bridge_addon/install/setup.bash' ]; then
    source /project-root/ros_gz_bridge_addon/install/setup.bash
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

