#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

gnome-terminal --title "cocel_handheld0" -- bash -c "roscore"
sleep 1

gnome-terminal --title "cocel_handheld1" -- bash -c "source /home/psh/workspace/ros_workspace/sensor_ws/devel/setup.bash && roslaunch livox_ros_driver livox_trigger.launch"

gnome-terminal --title "cocel_handheld2" -- bash -c "source /home/psh/workspace/ros_workspace/sensor_ws/devel/setup.bash && roslaunch vectornav vectornav.launch"

sleep 3
gnome-terminal --title "cocel_handheld3" -- bash -c "source /home/psh/workspace/ros_workspace/sensor_ws/devel/setup.bash && roslaunch spinnaker_camera_driver camera_trigger.launch"

sleep 3
gnome-terminal --title "cocel_handheld4" -- bash -c "source /home/psh/workspace/ros_workspace/sensor_ws/devel/setup.bash && roslaunch feedback_sensor_time run.launch"

source /home/psh/workspace/ros_workspace/handheld_ws/devel/setup.bash
roslaunch CoCEL_Handheld_DataRecorder run_multi_imu.launch & 
GUI_LAUNCH_PID=$!

wait $GUI_LAUNCH_PID
