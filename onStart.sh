#!/usr/bin/env bash
source /opt/ros/noetic/setup.sh
cd /workspaces/WRoverPlayground
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON