#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash
cd /home/kuka/catkin_ws3/
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
source /home/kuka/catkin_ws3/devel/setup.bash
sleep 20
roslaunch gakachu_backend start_server.launch
