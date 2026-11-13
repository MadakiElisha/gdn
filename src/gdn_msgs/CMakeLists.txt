cmake_minimum_required(VERSION 3.8)
project(gdn_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# R1.11 run log (eskf_node writes it, gdn_replay reads it; docs/ARCHITECTURE.md "Run log").
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RunInfo.msg"
  "msg/NavState.msg"
  "msg/NavUpdate.msg"
  "msg/NavEvent.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
