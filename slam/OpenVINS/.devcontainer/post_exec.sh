#!/bin/bash

# Ensure the catkin workspace and src directory exist
mkdir -p /workspace/catkin_ws/src

# Clone the ORB-SLAM3 repository if it hasn't been cloned already
if [ ! -d "/workspace/catkin_ws/src/OpenVINS" ]; then
    cd /workspace/catkin_ws/src
    echo "Cloning OpenVINS repository..."
    git clone https://github.com/iis-esslingen/OpenVINS.git
else
    echo "OpenVINS repository already exists."
fi

# Initialize and build the catkin workspace
cd /workspace/catkin_ws
source /opt/ros/noetic/setup.bash

# Run catkin build
echo "Building the catkin workspace..."
catkin build