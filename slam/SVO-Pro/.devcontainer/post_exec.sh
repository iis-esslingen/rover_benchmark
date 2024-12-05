#!/bin/bash

# Ensure the catkin workspace and src directory exist
mkdir -p /workspace/catkin_ws/src

# Clone the ORB-SLAM3 repository if it hasn't been cloned already
if [ ! -d "/workspace/catkin_ws/src/SVO-Pro" ]; then
    cd /workspace/catkin_ws/src
    echo "Cloning SVO-Pro repository..."
    git clone https://github.com/iis-esslingen/SVO-Pro.git

    catkin config --init --mkdirs --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3

    cd /workspace/catkin_ws/src/SVO-Pro/rpg_svo_pro_open/svo_online_loopclosing/vocabularies && ./download_voc.sh
else
    echo "SVO-Pro repository already exists."
fi

# Initialize and build the catkin workspace
cd /workspace/catkin_ws
source /opt/ros/noetic/setup.bash

# Run catkin build
echo "Building the catkin workspace..."
catkin build
