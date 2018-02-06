cmake_minimum_required(VERSION 2.8.3)
project(local_task_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	arm_kinematics
	arm_manipulation
	kuka_cv
	brics_actuator
	roscpp
	tf2_ros
	tf2
)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(camera_test test/CameraTest.cpp)
target_link_libraries(camera_test
	${catkin_LIBRARIES}
)