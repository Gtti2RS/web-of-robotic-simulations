Dependencies:
apt install -y ros-jazzy-gz-ros2-control ros-jazzy-control-msgs ros-jazzy-trajectory-msgs ros-jazzy-moveit-msgs
apt install ros-jazzy-ros2controlcli
apt install ros-jazzy-ros2-controllers ros-jazzy-joint-state-broadcaster
apt install ros-jazzy-ur
rclnodejs generate msgs
# for coppeliasim:
apt install ros-jazzy-moveit
apt install ros-jazzy-ur-moveit-config
apt install ros-jazzy-ur-robot-driver
sudo apt install python3-numpy python3-yaml python3-pydantic python3-toml python3-lxml
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH

Robot notes:
- currently only ur10_rg2 can be controlled, refer Assets/urdf/robots/ur10_rg2 to support more robots
- currently avoid_collision is not working, maybe due to wrong collision models, need to monitor the robot and return to a safe position when necessary
- currently the ros2 ur_controller has strange planning for the shoulder_pan_joints, sometimes it turns many rounds to reach the goal destination, in that case can trigger emergencyStop and moveToJoint: ("shoulder_pan_joint": null); When testing moveToCartesian, it's recommended to firstly call moveToJoint with "shoulder_pan_joint": null

Gazebo Stream notes:
- Stream only updates when simulation is running, if it's paused, streaming can be blank
- Use setEntityPose to set pose for wot_camera (or other name specified as CAMERA_NAME) to change view angle, initial pose: position:{x:-5,y:0,z:-5}, orientation:{roll:0, pitch: 40, yaw:0}
- Reset has known issue with dangling sensor, if the world file doesn't contain wot_camera, when you enable visualization and reset scene, you can still see the stream, but wot_camera won't be in the modelList and you can't adjust view angles; if you still need to adjust view angles after reset, you need to first exit then launch the simulation again (you can save the file before exit to preserve the sates).
- By the time of designing this framework, there is no out-of-box remote streaming from-browser solutions for gazebo harmonic, so I have adopted the camera_sensor + ros2 Image bridge + web_video_server approach. The image bridge results in high overhead with cpu usage, so the resolution has been set to 320*240 with 5 fps, using SceneBroadcaster for rendering should be more efficient, this could be future work.

file notes:
- gz_controller supports file upload in string formats, for direct file transfer with type: multipart/form-data, use fileUploader.js.
- file name is checked to avoid conflict with existing files
- saved files should not have conflict with existing files

Gazebo notes:
- User can use either sdf file or ros2 launch file for launching, in case using ros2 launch files, avoid trigger any process that requires gui if gazebo is running on headless server
- If user upload custom world files, plugins: Sensors, Physics, SceneBroadcaster, UserCommands should be included to enable all features, current files under Assets/gazebo/worlds/vendor originate from vendor folder "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds" and necessary plugins are added
- if spawning ur10_rg2 or world file has link to ur10_rg2, cofiguration on the robot will be started automatically

Ros2 utilities:
- publishMessage and sendRos2Cmds can be used to test basic ros2 features and ros2 communication across containers, sendRos2Cmds has timeout of 10s, if the cmds stucks for long it will be terminated, so instead publish to a topic continuously, use the --once flag

Coppeliasim notes:
- Must stop coppeliasim stream web page before starting coppeliasim, otherwise will get "address already in use" error

Using process_supervisor service:
- colcon build
- source ros_ws/install/setup.bash
- ros2 launch sim_process_supervisor supervisor.launch.py
//temporary process like node exec
- ros2 service call /process/exec sim_process_supervisor_interfaces/srv/ExecCmd "{cmd: 'ros2 topic list'}"
//managed long running processes
- ros2 service call /process/managed/list sim_process_supervisor_interfaces/srv/ManagedList "{}"
- ros2 service call /process/managed/start sim_process_supervisor_interfaces/srv/ManagedStart "{name: 'gz1', cmd: 'gz sim shapes.sdf'}"
- ros2 service call /process/managed/status sim_process_supervisor_interfaces/srv/ManagedStatus "{name: 'gz1'}"
- ros2 service call /process/managed/stop sim_process_supervisor_interfaces/srv/ManagedStop "{name: 'gz1'}"