#!/bin/bash

# export ROS_LOCALHOST_ONLY=1
export RMCS_ROBOT_TYPE=Dart_launcher

export ROS_DOMAIN_ID=10

# ROS Jazzy packages use the system Python 3.12.
unset PYTHONHOME
export PATH=/home/ws/.script:/usr/bin:/bin:/opt/ros/jazzy/bin:$PATH
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}

# Do not carry a stale workspace or Conda overlay into a fresh build.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash

# Keep the ROS prefix visible to CMake even when the shell had an old path.
export CMAKE_PREFIX_PATH=/opt/ros/jazzy${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}

if [ -f "/home/ws/install/setup.bash" ]; then
    source /home/ws/install/setup.bash
fi
