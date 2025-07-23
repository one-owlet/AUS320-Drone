#!/bin/bash

# Public environment variables for ROS
PUBLIC="source /opt/ros/noetic/setup.bash && \
        source /home/fast-drone/AUS320-Drone/devel/setup.bash && \
        export ROS_MASTER_URI=http://192.168.1.250:11311 && \
        export ROS_IP=192.168.1.250"

# Launch RealSenseD435 Camera
sleep 5s
{
    gnome-terminal --title "RealSenseD435" -- bash -c \
   "$PUBLIC && \
    roslaunch realsense2_camera rs_camera.launch; \
    exec bash"
}

# Launch MAVROS for PX4
sleep 10s
{
    gnome-terminal --title "PX4 Start" -- bash -c \
   "$PUBLIC && \
    sudo chmod 777 /dev/ttyACM0 && \
    roslaunch mavros px4.launch; \
    exec bash"
}&

# Launch VINS-Fusion for fast-drone-250
sleep 10s
{
    gnome-terminal --title "VINS-Fusion" -- bash -c \
   "$PUBLIC && \
    roslaunch vins fast_drone_250.launch; \
    exec bash"
}&

# View Odometry Data
sleep 5s
{
    gnome-terminal --title "odometry_viewer" -- bash -c \
   "$PUBLIC && \
    rostopic echo /vins_fusion/imu_propagate --noarr; \
    exec bash"
}&

# Launch PX4 Control
sleep 5s
{
    gnome-terminal --title "PX4 Control" -- bash -c \
   "$PUBLIC && \
    roslaunch px4ctrl run_ctrl.launch; \
    exec bash"
}&

wait
