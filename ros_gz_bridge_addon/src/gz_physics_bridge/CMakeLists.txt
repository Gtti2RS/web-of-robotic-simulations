cmake_minimum_required(VERSION 3.16)
project(gz_physics_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-transport CONFIG REQUIRED)
find_package(gz-msgs CONFIG REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPhysics.srv"
  "srv/GenerateWorldSdf.srv"
)

add_executable(set_physics_server src/set_physics_server.cpp)
ament_target_dependencies(set_physics_server rclcpp)

target_link_libraries(set_physics_server
  gz-transport::gz-transport
  gz-msgs::gz-msgs
)

# NEW: replace rosidl_target_interfaces(...) with this:
rosidl_get_typesupport_target(cpp_typesupport_target
  "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(set_physics_server "${cpp_typesupport_target}")

add_executable(generate_world_sdf_server src/generate_world_sdf_server.cpp)
ament_target_dependencies(generate_world_sdf_server rclcpp)

target_link_libraries(generate_world_sdf_server
  gz-transport::gz-transport
  gz-msgs::gz-msgs
  "${cpp_typesupport_target}"
)

install(TARGETS set_physics_server generate_world_sdf_server
  RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(stats_bridge src/stats_bridge.cpp)
ament_target_dependencies(stats_bridge rclcpp std_msgs)
target_link_libraries(stats_bridge
  gz-transport::gz-transport
  gz-msgs::gz-msgs)

install(TARGETS stats_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(pose_info_json_bridge src/pose_info_json_bridge.cpp)
ament_target_dependencies(pose_info_json_bridge rclcpp std_msgs)
target_link_libraries(pose_info_json_bridge
  gz-transport::gz-transport
  gz-msgs::gz-msgs)
install(TARGETS pose_info_json_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
