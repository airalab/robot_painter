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
    moveit_ros_move_group
    moveit_ros_visualization
    moveit_visual_tools
    moveit_msgs
    kuka_manipulation_moveit
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(camera_test test/CameraTest.cpp)
target_link_libraries(camera_test
    ${catkin_LIBRARIES}
)

add_executable(drawing_test test/DrawingTest.cpp)
target_link_libraries(drawing_test
    ${catkin_LIBRARIES}
)

add_executable(moveit_ik_test test/moveit_ik_test.cpp)
target_link_libraries(moveit_ik_test
    ${catkin_LIBRARIES}
)

add_executable(move_group_test test/move_group_test.cpp)
target_link_libraries(move_group_test
    ${catkin_LIBRARIES}
)

add_executable(move_group_assembly test/move_group_assembly_test.cpp)
target_link_libraries(move_group_assembly
    ${catkin_LIBRARIES}
)