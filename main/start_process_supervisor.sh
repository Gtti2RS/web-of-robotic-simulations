#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /project-root/ros2_ws/install/setup.bash
exec ros2 launch sim_process_supervisor supervisor.launch.py
