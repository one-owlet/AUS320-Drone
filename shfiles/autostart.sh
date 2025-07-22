#!/bin/bash

PUBLIC="source /opt/ros/noetic/setup.bash && \
        source /home/fast-drone/Fast-Drone-250-master/devel/setup.bash && \
        export ROS_MASTER_URI=http://192.168.1.250:11311 && \
        export ROS_IP=192.168.1.250"

sleep 5s
{
    gnome-terminal --title "PX4 Start" -- bash -c \
   "$PUBLIC && \
    sudo chmod 777 /dev/ttyACM0 && \
    roslaunch mavros px4.launch; \
    exec bash"
}&

wait
