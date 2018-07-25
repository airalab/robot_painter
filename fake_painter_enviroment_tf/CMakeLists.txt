cmake_minimum_required(VERSION 2.8.3)
project(fake_painter_enviroment_tf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  geometry_msgs
  kuka_cv
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_painter_enviroment_tf
#  CATKIN_DEPENDS geometry_msgs roscpp tf2 tf2_ros
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_broadcaster
	src/tf_broadcaster.cpp
)
target_link_libraries(tf_broadcaster
	${catkin_LIBRARIES}
)