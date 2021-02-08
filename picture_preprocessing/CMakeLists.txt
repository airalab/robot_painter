cmake_minimum_required(VERSION 2.8.3)
project(picture_preprocessing)

find_package(catkin REQUIRED
    rospy roslib
    nav_msgs geometry_msgs
    message_generation
)

add_service_files(
  FILES
  TextConverterService.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories()
