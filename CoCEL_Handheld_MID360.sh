#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

gnome-terminal --title "cocel_handheld0" -- bash -c "roscore"
sleep 1

gnome-terminal --title "cocel_handheld1" -- bash -c "source /home/psh/workspace/ros_workspace/sensor_ws/devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360.launch"

gnome-terminal --title "cocel_handheld2" -- bash -c "source /home/psh/workspace/ros_workspace/sensor_ws/devel/setup.bash && roslaunch vectornav vectornav.launch"

gnome-terminal --title "cocel_handheld3" -- bash -c "source /home/psh/workspace/ros_workspace/sensor_ws/devel/setup.bash && roslaunch spinnaker_camera_driver camera_trigger.launch"

source /home/psh/workspace/ros_workspace/handheld_ws/devel/setup.bash
roslaunch CoCEL_Handheld_DataRecorder run_multi_imu.launch & 
GUI_LAUNCH_PID=$!

wait $GUI_LAUNCH_PID
