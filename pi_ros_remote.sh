#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PARALLEL_JOBS=-j2 # Limit the number of compile threads due to memory limits
export LC_ALL=C
export ROS_MASTER_URI=http://PC.local:11311
export ROS_HOSTNAME=$(hostname).local
exec "$@"

