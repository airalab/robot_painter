cmake_minimum_required(VERSION 2.8.3)
project(kuka_cv)

find_package(catkin REQUIRED rospy roslib sensor_msgs geometry_msgs message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Palette.msg
  Colour.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RequestPalette.srv
  RequestCanvas.srv
  SetMode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)