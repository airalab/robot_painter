cmake_minimum_required(VERSION 2.8.3)
project(kuka_cv)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  message_generation
  roslib
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
#  LIBRARIES kuka_cv
	CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(CVTest tests/PainterCVTest.cpp src/CommonCV.cpp src/PainterCV.cpp)
target_link_libraries(CVTest ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${DEPENDENCIES})

add_executable(canvas_test tests/CanvasTest.cpp src/CommonCV.cpp)
target_link_libraries(canvas_test ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(palette_test tests/PaletteTest.cpp src/CommonCV.cpp src/PainterCV.cpp)
target_link_libraries(palette_test ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${DEPENDENCIES})