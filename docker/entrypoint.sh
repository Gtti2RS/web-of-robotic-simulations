#!/usr/bin/env bash
set -e

# Source ROS 2
source /opt/ros/jazzy/setup.bash

cd /usr/src/app

# Make sure node_modules exist
if [ ! -d node_modules ]; then
  npm ci || npm install
fi

# Rebuild rclnodejs to match ROS & Node ABI
npm rebuild rclnodejs --build-from-source || true

exec "$@"
