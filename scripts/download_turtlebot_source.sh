#!/bin/bash

# Helper script for downloading the turtlebot source code.
cd ~
mkdir -p ~/turtlebot2i/src
cd ~/turtlebot2i/src
git clone https://github.com/Interbotix/turtlebot2i.git -b turtlebot2i_old .
git clone https://github.com/Interbotix/arbotix_ros.git -b turtlebot2i
git clone https://github.com/Interbotix/phantomx_pincher_arm.git -b turtlebot2i_old
git clone https://github.com/Interbotix/ros_astra_camera -b filterlibrary
git clone https://github.com/Interbotix/ros_astra_launch
