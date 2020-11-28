#!/bin/bash

# A wrapper around CMake so that it works with catkin but behaves
# nicely with CLion.source /opt/ros/kinetic/setup.bash

set -e

# Load ROS environment.
source /opt/ros/kinetic/setup.bash

cmake "$@"
