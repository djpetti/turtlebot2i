#!/bin/bash

set -e

source ~/.bashrc
source devel/setup.sh
# TODO (daniel) Investigate why this gets set wrong by setup.sh.
export TURTLEBOT_STACKS=interbotix

roslaunch turtlebot2i_bringup turtlebot2i_explore.launch