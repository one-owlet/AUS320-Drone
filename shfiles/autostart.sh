#!/bin/bash

# Public environment variables for ROS
PUBLIC="source /opt/ros/noetic/setup.bash && \
        source /home/fast-drone/AUS320-Drone/devel/setup.bash && \
        export ROS_MASTER_URI=http://192.168.1.250:11311 && \
        export ROS_IP=192.168.1.250"

# Launch FAST-LIO2
sleep 5s
{
    gnome-terminal --title "FAST-LIO2" -- bash -c \
   "$PUBLIC && \
    roslaunch fast_lio mapping_mid360.launch; \
    exec bash"
}&

# View Odometry Data

# CLI command to view odometry data
sleep 5s
{
    gnome-terminal --title "odometry_viewer_CLI" -- bash -c \
   "$PUBLIC && \
    rostopic echo /Odom_high_freq --noarr; \
    exec bash"
}&

# GUI command to view odometry data
sleep 5s
{
    gnome-terminal --title "odometry_viewer_GUI" -- bash -c \
   "$PUBLIC && \
    rosrun rqt_plot rqt_plot /Odom_high_freq/pose/pose/position; \
    exec bash"
}&

# # Launch EGO-Planner
# sleep 5s
# {
#     gnome-terminal --title "EGO-Planner" -- bash -c \
#    "$PUBLIC && \
#     roslaunch ego_planner single_run_in_exp.launch; \
#     exec bash"
# }&

# Launch MAVROS for PX4
sleep 5s
{
    gnome-terminal --title "PX4 Start" -- bash -c \
   "$PUBLIC && \
    sudo chmod 777 /dev/ttyACM0 && \
    roslaunch mavros px4.launch; \
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
