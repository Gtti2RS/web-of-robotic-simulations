#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /project-root/install/setup.bash

# Start robot_state_publisher with UR10 with RG2 URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat /project-root/resource/model/ur10_with_rg2.urdf)" \
  -p use_sim_time:=true
