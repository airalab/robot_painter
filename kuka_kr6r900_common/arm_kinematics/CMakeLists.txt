cmake_minimum_required(VERSION 2.8.3)
project(arm_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES arm_kinematics
)

include_directories(
    include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/KR6ArmKinematics.cpp
)

add_executable(test_k test/KinematicTest.cpp src/KR6ArmKinematics.cpp)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)