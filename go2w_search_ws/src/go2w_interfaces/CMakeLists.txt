cmake_minimum_required(VERSION 3.5)
project(go2w_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SearchArea.msg"
  "msg/Waypoint.msg"
  "msg/PathPlan.msg"
  "msg/TargetDetection.msg"
  "msg/MissionStatus.msg"
  "msg/RobotState.msg"
  "msg/MissionReport.msg"
  "srv/StartSearch.srv"
  "srv/StopSearch.srv"
)

ament_package()
