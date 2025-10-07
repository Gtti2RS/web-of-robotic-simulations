#!/bin/bash
# Script to run inside gz_sim container
source /opt/ros/jazzy/setup.bash && \
source /project-root/install/setup.bash && \
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat /project-root/Assets/urdf/examples/robots/ur10_rg2/ur10_rg2.urdf)" \
  -p use_sim_time:=true  &\
ros2 param set /controller_manager use_sim_time true && \
ros2 param set /robot_state_publisher use_sim_time true && \
ros2 control load_controller joint_state_broadcaster && \
ros2 control load_controller ur_arm_controller && \
ros2 control load_controller rg2_trajectory_controller && \
ros2 param load /controller_manager /project-root/Assets/urdf/examples/robots/ur10_rg2/ur10_controllers.yaml && \
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController '{name: joint_state_broadcaster}' && \
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController '{name: ur_arm_controller}' && \
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController '{name: rg2_trajectory_controller}' && \
ros2 control set_controller_state joint_state_broadcaster active && \
ros2 control set_controller_state ur_arm_controller active && \
ros2 control set_controller_state rg2_trajectory_controller active && \
echo 'UR10 configuration complete! starting moveit...'   && \
ros2 launch ur_moveit_config ur_moveit.launch.py   ur_type:=ur10   launch_rviz:=false
