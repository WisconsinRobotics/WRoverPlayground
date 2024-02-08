#!/usr/bin/env bash

source /opt/ros/noetic/setup.sh

# Build WRoverPlayground code
cd /workspaces/WRoverPlayground
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Create virtual environment if it does not exist
if [ ! -d /workspaces/WRoverPlayground/venv ]; then
  python3 -m venv venv
fi

# Install python dependencies for robot_sim_gui
source /workspaces/WRoverPlayground/venv/bin/activate
python3 -m pip install -U pip
pip install -r requirements.txt
