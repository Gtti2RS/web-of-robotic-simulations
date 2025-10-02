
apt install -y ros-jazzy-gz-ros2-control ros-jazzy-control-msgs ros-jazzy-trajectory-msgs ros-jazzy-moveit-msgs
apt install ros-jazzy-ros2controlcli
apt install ros-jazzy-ros2-controllers ros-jazzy-joint-state-broadcaster
apt install ros-jazzy-ur
rclnodejs generate msgs

currently avoid_collision is not working, maybe due to wrong collision models, need to monitor the robot and return to a safe position when necessary