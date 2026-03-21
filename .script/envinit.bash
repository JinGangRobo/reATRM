#!/bin/bash

# export ROS_LOCALHOST_ONLY=1
export RMCS_ROBOT_TYPE=mec-hero

export ROS_DOMAIN_ID=10

source /opt/ros/jazzy/setup.bash

if [ -f "/home/ws/install/setup.bash" ]; then
    source /home/ws/install/setup.bash
fi