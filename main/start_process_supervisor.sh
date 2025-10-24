#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /opt/ros_gz_bridge_addon/install/setup.bash
exec ros2 launch sim_process_supervisor supervisor.launch.py
