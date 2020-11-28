#!/bin/bash

# Helper script for running Rviz inside Docker.

set -e

# Allow Rviz to resolve the turtlebot hostname.
echo "${ROS_IP} turtlebot" >> /etc/hosts

cd /home/turtlebot/turtlebot2i
source devel/setup.sh
rosrun rviz rviz -d src/turtlebot2i_bringup/rviz/remote_view.rviz
