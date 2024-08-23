#!/usr/bin/env bash

gnome-terminal --title "cocel_handheld_fastlio" -- bash -c "source /home/psh/workspace/ros_workspace/lio_ws/devel/setup.bash && roslaunch fast_lio mapping_avia.launch"
