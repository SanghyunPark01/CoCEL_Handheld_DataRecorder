#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

gnome-terminal --title "cocel_handheld0" -- bash -c "roscore"
sleep 1

gnome-terminal --title "cocel_handheld1" -- bash -c "source /mnt/HDD1/ros_workspace/livox_trigger_ws/devel/setup.bash && roslaunch livox_ros_driver livox_lidar_trigger.launch"

gnome-terminal --title "cocel_handheld2" -- bash -c "source /mnt/HDD1/ros_workspace/ws_imu/devel/setup.bash && roslaunch vectornav vectornav.launch"

gnome-terminal --title "cocel_handheld3" -- bash -c "source /mnt/HDD1/ros_workspace/flir_ws/devel/setup.bash && roslaunch spinnaker_camera_driver camera_trigger.launch"

source /mnt/HDD1/ros_workspace/test_ws/devel/setup.bash
roslaunch CoCEL_Handheld_DataRecorder run_full.launch & 
GUI_LAUNCH_PID=$!

wait $GUI_LAUNCH_PID
