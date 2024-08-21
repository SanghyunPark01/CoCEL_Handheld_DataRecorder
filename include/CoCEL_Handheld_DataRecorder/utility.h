#ifndef UTILITY_H
#define UTILITY_H

#include <signal.h>
#include <thread>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>

#include <livox_ros_driver/CustomMsg.h>
#include "CoCEL_Handheld_DataRecorder/record_flag.h"
#include "CoCEL_Handheld_DataRecorder/save_bag_path.h"

void mySigintHandler(int sig){
    ros::shutdown();
}

#endif UTILITY_H