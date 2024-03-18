#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /home/gerardo/rescue_ws/devel/setup.bash

sudo chmod 666 /dev/ttyUSB0

# Launch RPLeader ROS view
gnome-terminal -- bash -c "roslaunch rplidar_ros rplidar.launch; exec bash"

# Wait a bit for the first launch to initialize
sleep 5

# Launch Hector SLAM tutorial
gnome-terminal -- bash -c "roslaunch hector_slam_launch tutorial.launch; exec bash"


