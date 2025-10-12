cmake_minimum_required(VERSION 3.16)
project(sim_process_supervisor_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartCmd.srv"
  "srv/ExecCmd.srv"
  "srv/ManagedStart.srv"
  "srv/ManagedStop.srv"
  "srv/ManagedStatus.srv"
  "srv/ManagedList.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
