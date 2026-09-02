cmake_minimum_required(VERSION 3.8)
project(platoon_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/LaneInfo.msg"
  "msg/VehicleCmd.msg"
  "msg/Telemetry.msg"
  "msg/FsmCmd.msg"
  "msg/V2xTarget.msg"
  "msg/V2xTargets.msg"
  "msg/SelfStatus.msg"

)

ament_export_dependencies(rosidl_default_runtime)

ament_package()