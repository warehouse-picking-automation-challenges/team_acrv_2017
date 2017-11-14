#!/bin/bash

# After clean install first run: sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
# Then run: sudo reboot
# MAKE SURE NO REALSENSE IS PLUGGED INTO YOUR COMPUTER
# Make sure acrv_apc_2017 is cloned into ~/ros_ws/src/

sudo apt-get install -y ntp
sudo apt-get install -y ntpdate

# ROS install section
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full
if [ ! -e "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update
sudo apt-get install -y libssl-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libnlopt-dev
sudo apt-get install -y libxmlrpc-c++8-dev
sudo apt-get install -y libudev-dev
sudo apt-get install -y libusb-1.0-0-dev
sudo apt-get install -y pkg-config
sudo apt-get install -y libglfw3-dev
sudo apt-get install -y git-core
sudo apt-get install -y git
sudo apt-get install -y cmake
sudo apt-get install -y python-argparse
sudo apt-get install -y python-wstool
sudo apt-get install -y python-vcstools
sudo apt-get install -y python-rosdep
sudo apt-get install -y python-sklearn
sudo apt-get install -y python-termcolor
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-kinetic-pcl-conversions
sudo apt-get install -y ros-kinetic-ar-track-alvar
sudo apt-get install -y ros-kinetic-image-transport
sudo apt-get install -y ros-kinetic-opencv3
sudo apt-get install -y ros-kinetic-position-controller
sudo apt-get install -y ros-kinetic-joint-trajectory-controller
sudo apt-get install -y ros-kinetic-joint-state-controller
sudo apt-get install -y ros-kinetic-rosserial-python
sudo apt-get install -y ros-kinetic-control-msgs
sudo apt-get install -y ros-kinetic-joystick-drivers
sudo apt-get install -y ros-kinetic-gazebo-ros-control
sudo apt-get install -y ros-kinetic-effort-controllers
sudo apt-get install -y ros-kinetic-moveit-*
sudo apt-get install -y ros-kinetic-tf2
sudo apt-get install -y ros-kinetic-ros-control
sudo apt-get install -y ros-kinetic-trac-ik
# sudo apt-get install -y ros-kinetic-librealsense

. "/opt/ros/kinetic/setup.bash"

## Not needed anymore due to custom version being used
# Librealsense stuff
# mkdir -p "${HOME}/co"
# cd "${HOME}/co"
# git clone https://github.com/IntelRealSense/librealsense.git
# cd "./librealsense"
# mkdir build
# cd "./build"
# cmake ../ -DBUILD_EXAMPLES=true
# make
# sudo make install
# cd ../
# sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
# sudo udevadm control --reload-rules && udevadm trigger
# ./scripts/patch-realsense-ubuntu-xenial.sh

# Build cv_bridge from source
# Has to be done due to cv_bridge from ros-kinetic packages wanting OpenCV3-3.1.0
# But OpenCV from ros-kinetic packages is 3.2.0
mkdir -p "${HOME}/ros_ws/src/"
cd "${HOME}/ros_ws/src"
git clone https://github.com/ros-perception/vision_opencv.git
mkdir "./vision_opencv/cv_bridge/build"
cd "./vision_opencv/cv_bridge/build"
cmake ..
make
sudo make install

# PCL stuff
mkdir -p "${HOME}/co"
cd "${HOME}/co"
git clone https://github.com/PointCloudLibrary/pcl.git
cd "./pcl"
git checkout tags/pcl-1.8.0 -b local-1.8.0
mkdir -p "./build"
cd "./build"
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_GPU=ON -DBUILD_CUDA=ON
make -j3
sudo make install

# ACRV_APC stuff
# sudo apt-get install -y libeigen3-dev libnlopt-dev libxmlrpc-c++8-dev libudev-dev python-sklearn python-termcolor
cd "${HOME}/ros_ws/src"
git clone https://bitbucket.org/acrv/acrv_apc_2017.git
git clone https://github.com/code-iai/iai_robots.git
# Kinect will run off of Juxi's nuc. iai_kinect2 requires OpenCV2 which is not available for ros-kinetic

# Install the segmentatioin library.
cd "./acrv_apc_2017"
git checkout cartesian
cd "./segmentation"
mkdir build
cd build
cmake ..
make
sudo make install

# Convenience things
# Build and profile the ROS workspace
cd "${HOME}/ros_ws"
catkin_make -j3
rospack profile

# Scales
sudo apt-get install -y libhidapi-dev

# Dynamixels
sudo apt-get install -y ros-kinetic-dynamixel-*
