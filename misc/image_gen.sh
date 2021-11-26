#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash
cd /home/kuka/catkin_ws3/
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
source /home/kuka/catkin_ws3/devel/setup.bash
sleep 5
roslaunch image_generation image_generation.launch