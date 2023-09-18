#!/usr/bin/env bash
source /opt/ros/noetic/setup.sh

if [ ! -d /workspaces/WRoverPlayground/venv ]; then
  python3 -m venv venv
fi

source /workspaces/WRoverPlayground/venv/bin/activate
pip install -r requirements.txt

cd /workspaces/WRoverPlayground
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON