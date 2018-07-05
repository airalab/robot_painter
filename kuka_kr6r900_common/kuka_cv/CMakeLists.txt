cmake_minimum_required(VERSION 2.8.3)
project(kuka_cv)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  message_generation
  roslib
  tf2_ros
  geometry_msgs
)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
set(DEPENDENCIES realsense)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pose.msg
  Color.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RequestPalette.srv
  RequestCanvas.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package (
#  INCLUDE_DIRS include
  LIBRARIES kuka_cv
	CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# Create kuka_cv library
add_library(${PROJECT_NAME}
  src/CommonCV.cpp src/CameraTransforms.cpp src/PainterCV.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${DEPENDENCIES}
)

add_executable(CVTest tests/PainterCVTest.cpp)
target_link_libraries(CVTest ${PROJECT_NAME})

add_executable(canvas_test tests/CanvasTest.cpp)
target_link_libraries(canvas_test ${PROJECT_NAME})

add_executable(palette_test tests/PaletteTest.cpp)
target_link_libraries(palette_test ${PROJECT_NAME})

add_executable(br_test tests/broadcaster_test.cpp)
target_link_libraries(br_test ${PROJECT_NAME})

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