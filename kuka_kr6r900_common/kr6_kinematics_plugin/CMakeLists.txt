cmake_minimum_required(VERSION 2.8.3)
project(kr6_kinematics_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    arm_kinematics
    geometry_msgs
    moveit_core
    moveit_ros_planning
    pluginlib
    urdf
)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES ${PROJECT_NAME}
    # CATKIN_DEPENDS
    #     arm_kinematics
    #     geometry_msgs
    #     moveit_core
    #     pluginlib
    #     urdf
    # DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/KR6KinematicsPlugin.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES kr6_kinematics_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)