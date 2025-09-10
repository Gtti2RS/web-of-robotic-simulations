#!/bin/bash
# Script to run inside gz_sim container
source /opt/ros/jazzy/setup.bash && \
ros2 param set /controller_manager use_sim_time true && \
ros2 param set /robot_state_publisher use_sim_time true && \
ros2 control load_controller joint_state_broadcaster && \
ros2 control load_controller ur_arm_controller && \
ros2 control load_controller rg2_trajectory_controller && \
ros2 param load /controller_manager /project-root/resource/model/ur_description/ur10_controllers.yaml && \
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController '{name: joint_state_broadcaster}' && \
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController '{name: ur_arm_controller}' && \
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController '{name: rg2_trajectory_controller}' && \
ros2 control set_controller_state joint_state_broadcaster active && \
ros2 control set_controller_state ur_arm_controller active && \
ros2 control set_controller_state rg2_trajectory_controller active && \
echo 'UR10 configuration complete!'   