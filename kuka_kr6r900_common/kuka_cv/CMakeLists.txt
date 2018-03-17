cmake_minimum_required(VERSION 2.8.3)
project(kuka_cv)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(OpenCV REQUIRED)
set(DEPENDENCIES realsense)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuka_cv
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(FirstTest tests/FirstTets.cpp src/CommonCV.cpp)
target_link_libraries(FirstTest ${OpenCV_LIBS} ${DEPENDENCIES})
add_executable(CalcWidth tests/CalcWidth.cpp src/CommonCV.cpp)
target_link_libraries(CalcWidth ${OpenCV_LIBS})