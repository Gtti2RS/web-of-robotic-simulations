cmake_minimum_required(VERSION 3.16)
project(gz_physics_bridge_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Declare interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPhysics.srv"
  "msg/WorldStatistics.msg"
  "msg/LogPlaybackStatistics.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Export runtime dependency so downstream users can find generated headers/types
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(builtin_interfaces std_msgs)

ament_package()
