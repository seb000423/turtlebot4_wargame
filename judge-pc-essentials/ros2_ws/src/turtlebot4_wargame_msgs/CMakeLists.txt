cmake_minimum_required(VERSION 3.8)
project(turtlebot4_wargame_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HitEvent.msg"
  "msg/Detection.msg"
  "msg/Detections.msg"
  "msg/TeamStatus.msg"
)

ament_package()
