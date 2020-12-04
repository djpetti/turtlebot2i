#!/bin/bash

# Shell script for setting up the ROS environment correctly. This is common
# functionality between the Docker build and the actual robot.
# Meant to be used with a fresh OS install.

set -e

apt-get update
apt-get install -y build-essential libgtk-3-dev

# Add the ROS PPA.
apt-get install -y lsb-release
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-get update

# Install ROS.
apt-get install -y --allow-unauthenticated git ros-kinetic-desktop
rm -f /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
# Rosdep doesn't like to be run as root.
runuser -l turtlebot -c "rosdep update"

# Install RealSense package.
# This is a terribly hacky way suggested on Github to get around this issue:
# https://github.com/IntelRealSense/librealsense/issues/4781
# Install Dependencies needed by librealsense. `deb -i` will not resolve these
apt-get install -y binutils cpp cpp-5 dkms fakeroot gcc gcc-5 kmod libasan2 libatomic1 libc-dev-bin libc6-dev libcc1-0 libcilkrts5 libfakeroot libgcc-5-dev libgmp10 libgomp1 libisl15 libitm1 liblsan0 libmpc3 libmpfr4 libmpx0 libquadmath0 libssl-dev libssl-doc libtsan0 libubsan0 libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic linux-libc-dev make manpages manpages-dev menu patch zlib1g-dev
apt-get install -y libssl-dev libssl-doc libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic zlib1g-dev

# Modify librealsense deb (unpack, replace script, repack)
apt-get install -y wget
apt-get download ros-kinetic-librealsense
dpkg-deb -R ros-kinetic-librealsense*.deb ros-rslib/

wget https://gist.githubusercontent.com/dizz/404ef259a15e1410d692792da0c27a47/raw/3769e80a051b5f2ce2a08d4ee6f79c766724f495/postinst
chmod +x postinst
cp postinst ros-rslib/DEBIAN
dpkg-deb -b ./ros-rslib/ ros-kinetic-librealsense_1.12.1-0xenial-20190830_icrlab_amd64.deb

# Install container friendly librealsense
dpkg -i ros-kinetic-librealsense_1.12.1-0xenial-20190830_icrlab_amd64.deb

# lock from updates
apt-mark hold ros-kinetic-librealsense
# Clean up old debs.
rm ros-kinetic-librealsense*.deb

apt-get install -y ros-kinetic-realsense-camera

# Compile RealSense library.
apt-get install -y libglfw3-dev
cd /tmp/librealsense
mkdir -p /tmp/build && cd /tmp/build
cmake /tmp/librealsense
make -j $(nproc) && make install
cd ~

# Install turtlebot packages.
apt-get install -y ros-kinetic-turtlebot* libudev-dev ros-kinetic-find-object-2d ros-kinetic-rtabmap-ros \
  ros-kinetic-moveit ros-kinetic-octomap-ros ros-kinetic-manipulation-msgs ros-kinetic-controller-manager \
  python-wxgtk3.0 ros-kinetic-tf2-geometry-msgs libhdf5-openmpi-dev

# Clear the apt cache at the end, so this doesn't get too big if we're running
# in Docker.
rm -rf /var/lib/apt/lists/*
