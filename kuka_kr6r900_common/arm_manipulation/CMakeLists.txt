cmake_minimum_required(VERSION 2.8.3)
project(arm_manipulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    arm_kinematics
    brics_actuator
    roscpp
    sensor_msgs
    brics_actuator
	kuka_kr6_support
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES arm_manipulation
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/Manipulator.cpp
)

add_executable(move_arm src/MoveArm.cpp src/Manipulator.cpp)
target_link_libraries(move_arm
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)