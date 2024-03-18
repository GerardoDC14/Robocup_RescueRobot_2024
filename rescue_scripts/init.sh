#!/bin/bash

# Source env.
source /opt/ros/noetic/setup.bash
source /home/gerardo/rescue_ws/devel/setup.bash

sudo chmod 666 /dev/ttyUSB0

# Launch rplidar 
gnome-terminal -- bash -c "roslaunch rplidar_ros rplidar.launch; exec bash"

sleep 5

# Launch Hector SlAM
gnome-terminal -- bash -c "roslaunch hector_slam_launch tutorial.launch; exec bash"


