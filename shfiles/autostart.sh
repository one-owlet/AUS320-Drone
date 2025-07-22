#!/bin/bash

PUBLIC="source /opt/ros/noetic/setup.bash && \
        source /home/fast-drone/AUS320-Drone/devel/setup.bash && \
        export ROS_MASTER_URI=http://192.168.1.250:11311 && \
        export ROS_IP=192.168.1.250"

sleep 5s
{
    gnome-terminal --title "FAST-LIO2" -- bash -c \
   "$PUBLIC && \
    roslaunch fast_lio mapping_mid360.launch; \
    exec bash"
}&

sleep 5s
{
    gnome-terminal --title "odometry_viewr" -- bash -c \
   "$PUBLIC && \
    rostopic echo /Odom_high_freq; \
    exec bash"
}&

sleep 2s
{
    gnome-terminal --title "PX4 Start" -- bash -c \
   "$PUBLIC && \
    sudo chmod 777 /dev/ttyACM0 && \
    roslaunch mavros px4.launch; \
    exec bash"
}&

sleep 5s
{
    gnome-terminal --title "PX4 Control" -- bash -c \
   "$PUBLIC && \
    roslaunch px4ctrl run_ctrl.launch; \
    exec bash"
}&

wait
