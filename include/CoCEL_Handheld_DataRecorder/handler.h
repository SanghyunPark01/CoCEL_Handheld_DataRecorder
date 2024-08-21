#ifndef HANDLER_H
#define HANDLER_H

#include "CoCEL_Handheld_DataRecorder/utility.h"

class Handler
{
private:
    // param
    std::string IMU_TOPIC, LIDAR_TOPIC, CAM_TOPIC;
    std::string IMU_TOPIC_2 = "";
    int IMU_HZ, LIDAR_HZ, CAM_HZ;
    bool _mbImgIsCompressed = false;
    bool _mbMultiIMU = false;
    
    // ROS
    ros::NodeHandle _nh;
    ros::Subscriber _mSubIMU;
    ros::Subscriber _mSubIMU2;
    ros::Subscriber _mSubLiDAR;
    ros::Subscriber _mSubImg;
    ros::ServiceServer _mSrvRecord;
    ros::ServiceServer _mSrvSavePath;

    // ROS Callback
    void callbackIMU(const sensor_msgs::Imu::ConstPtr &msgIMU);
    void callbackIMU2(const sensor_msgs::Imu::ConstPtr &msgIMU);
    void callbackLiDAR(const livox_ros_driver::CustomMsg::ConstPtr &msgLiDAR);
    void callbackImage(const sensor_msgs::ImageConstPtr &msgIMG);
    void callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr &msgIMG);
    bool setRecordFlag(CoCEL_Handheld_DataRecorder::record_flagRequest& req, CoCEL_Handheld_DataRecorder::record_flagResponse& res);
    bool setSavePath(CoCEL_Handheld_DataRecorder::save_bag_pathRequest& req, CoCEL_Handheld_DataRecorder::save_bag_pathResponse& res);

    // ROS Data
    std::queue<sensor_msgs::Imu::ConstPtr> _mqIMUBuf;
    std::mutex _mtxCallbackIMU;
    std::queue<sensor_msgs::Imu::ConstPtr> _mqIMU2Buf;
    std::mutex _mtxCallbackIMU2;
    std::queue<livox_ros_driver::CustomMsg::ConstPtr> _mqLiDARBuf;
    std::mutex _mtxCallbackLiDAR;
    std::queue<sensor_msgs::ImageConstPtr> _mqImgBuf;
    std::mutex _mtxCallbackImg;
    std::queue<sensor_msgs::CompressedImageConstPtr> _mqCImgBuf;
    std::mutex _mtxCallbackCImg;

    // Flag
    bool _mbRecordFlag = false;
    bool _mbSaveFlag = false;
    std::string _msRecordFlag = "";
    std::string _msSavePath = "";
    std::string _msRecordFlag__ = "";

    // record
    ros::Time _mrtStartRecordTime;
    rosbag::Bag* _mptrRosbag = nullptr;
    void recordData(std::string sMsgFlag);
    
    // save
    ros::Time _mrtEndRecordTime;
    void saveData(void);

public:
    Handler(const ros::NodeHandle& nh_);
    void processHandler(void);
};

#endif HANDLER_H