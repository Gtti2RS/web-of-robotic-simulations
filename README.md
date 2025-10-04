
apt install -y ros-jazzy-gz-ros2-control ros-jazzy-control-msgs ros-jazzy-trajectory-msgs ros-jazzy-moveit-msgs
apt install ros-jazzy-ros2controlcli
apt install ros-jazzy-ros2-controllers ros-jazzy-joint-state-broadcaster
apt install ros-jazzy-ur
rclnodejs generate msgs

currently avoid_collision is not working, maybe due to wrong collision models, need to monitor the robot and return to a safe position when necessary

stream only updates when simulation is running, if it's paused, streaming can be blank
for gazebo, reset has known issue with dangling sensor, if the world file doesn't contain wot_camera, when you enable visualization and reset scene, you can still see the stream, but wot_camera won't be in the modelList and you can't adjust view angles; if you still need to adjust view angles after reset, you need to first exit then launch the simulation again (you can save the file before exit to preserve the sates).