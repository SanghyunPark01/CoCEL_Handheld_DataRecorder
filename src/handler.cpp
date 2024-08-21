#include "CoCEL_Handheld_DataRecorder/handler.h"

Handler::Handler(const ros::NodeHandle& nh_):_nh(nh_)
{
    _nh.param<std::string>("/sensor/imu_topic", IMU_TOPIC, "/");
    _nh.param<std::string>("/sensor/lidar_topic", LIDAR_TOPIC, "/");
    _nh.param<std::string>("/sensor/cam_topic", CAM_TOPIC, "/");
    _nh.param<std::string>("/sensor/imu_topic2", IMU_TOPIC_2, "");

    if(IMU_TOPIC_2.length() >= 2) _mbMultiIMU = true;

    // IMU_TOPIC = "/cocel_handheld/imu";
    // LIDAR_TOPIC = "/cocel_handheld/lidar";
    // CAM_TOPIC = "/cocel_handheld/camera";

    _nh.param<int>("/setting/imu_hz", IMU_HZ, 200);
    _nh.param<int>("/setting/lidar_hz", LIDAR_HZ, 10);
    _nh.param<int>("/setting/cam_hz", CAM_HZ, 50);

    _nh.param<bool>("/setting/img_is_compressed", _mbImgIsCompressed, false);

    if(_mbImgIsCompressed)
    {
        _mSubImg = _nh.subscribe(CAM_TOPIC, 1000, &Handler::callbackImageCompressed, this);
    }
    else
    {
        _mSubImg = _nh.subscribe(CAM_TOPIC, 1000, &Handler::callbackImage, this);
    }
    _mSubIMU = _nh.subscribe(IMU_TOPIC, 1000, &Handler::callbackIMU, this);
    if(_mbMultiIMU)
    {
        _mSubIMU2 = _nh.subscribe(IMU_TOPIC_2, 1000, &Handler::callbackIMU2, this);
    }
    _mSubLiDAR = _nh.subscribe(LIDAR_TOPIC, 1000, &Handler::callbackLiDAR, this);
    _mSrvRecord = _nh.advertiseService("/cocel_handheld/record_flag/541635134", &Handler::setRecordFlag, this);
    _mSrvSavePath = _nh.advertiseService("/cocel_handheld/save_flag/846158586", &Handler::setSavePath, this);

    
}

/* @@@@@@@@@@@@@@@@@@@@@
@@@@@@@ Callback @@@@@@@
@@@@@@@@@@@@@@@@@@@@@ */

void Handler::callbackIMU(const sensor_msgs::Imu::ConstPtr &msgIMU)
{
    if(!_mbRecordFlag)return;

    std::unique_lock<std::mutex> lock(_mtxCallbackIMU);
    _mqIMUBuf.push(msgIMU);
    // if(_mqIMUBuf.size() > (0.5*(double)IMU_HZ))_mqIMUBuf.pop();
}
void Handler::callbackIMU2(const sensor_msgs::Imu::ConstPtr &msgIMU)
{
    if(!_mbRecordFlag)return;

    std::unique_lock<std::mutex> lock(_mtxCallbackIMU2);
    _mqIMU2Buf.push(msgIMU);
}
void Handler::callbackLiDAR(const livox_ros_driver::CustomMsg::ConstPtr &msgLiDAR)
{
    if(!_mbRecordFlag)return;

    std::unique_lock<std::mutex> lock(_mtxCallbackLiDAR);
    _mqLiDARBuf.push(msgLiDAR);
    // if(_mqLiDARBuf.size() > (0.5*(double)LIDAR_HZ))_mqLiDARBuf.pop();
}
void Handler::callbackImage(const sensor_msgs::ImageConstPtr &msgIMG)
{
    if(!_mbRecordFlag)return;

    std::unique_lock<std::mutex> lock(_mtxCallbackImg);
    _mqImgBuf.push(msgIMG);
    // if(_mqImgBuf.size() > (0.5*(double)CAM_HZ))_mqImgBuf.pop();
}
void Handler::callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr &msgIMG)
{
    if(!_mbRecordFlag)return;

    std::unique_lock<std::mutex> lock(_mtxCallbackCImg);
    _mqCImgBuf.push(msgIMG);
    // if(_mqCImgBuf.size() > (0.5*(double)CAM_HZ))_mqCImgBuf.pop();
}

/* @@@@@@@@@@@@@@@@@@@@@
@@@@@@@ Service @@@@@@@
@@@@@@@@@@@@@@@@@@@@@ */

bool Handler::setRecordFlag(CoCEL_Handheld_DataRecorder::record_flagRequest& req, CoCEL_Handheld_DataRecorder::record_flagResponse& res)
{
    _msRecordFlag = req.flag;
    if(_msRecordFlag != "000")
    {
        _msRecordFlag__ = _msRecordFlag;
    }
    _mbSaveFlag = false;
    _mrtStartRecordTime = ros::Time::now();

    // _mqIMUBuf = std::queue<sensor_msgs::Imu::ConstPtr>();
    // _mqLiDARBuf = std::queue<livox_ros_driver::CustomMsg::ConstPtr>();
    // _mqImgBuf = std::queue<sensor_msgs::ImageConstPtr>();
    // _mqCImgBuf = std::queue<sensor_msgs::CompressedImageConstPtr>();


    _mbRecordFlag = true;
    res.status = true;
    return true;
}
bool Handler::setSavePath(CoCEL_Handheld_DataRecorder::save_bag_pathRequest& req, CoCEL_Handheld_DataRecorder::save_bag_pathResponse& res)
{
    _msSavePath = req.path;
    try
    {
        _mptrRosbag = new rosbag::Bag();
        _mptrRosbag->open(_msSavePath, rosbag::bagmode::Write);
    }
    catch (const rosbag::BagException& e)
    {
        delete _mptrRosbag;
        res.status = false;
        return true;
    }
    res.status = true;
    return true;
}

/* @@@@@@@@@@@@@@@@@@@
@@@@@@@ Thread @@@@@@@
@@@@@@@@@@@@@@@@@@@ */

void Handler::processHandler(void)
{
    while (1)
    {
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
        if(!_mbRecordFlag)continue;

        // Record
        recordData(_msRecordFlag);

        // Save
        if(_mbSaveFlag){
            saveData();
        }
        
    }
}

/* @@@@@@@@@@@@@@@@@@@@@
@@@@@@ For Thread @@@@@@
@@@@@@@@@@@@@@@@@@@@@ */

void Handler::recordData(std::string sMsgFlag)
{
    if(sMsgFlag == "000")
    {
        _mbSaveFlag = true;
        _mrtEndRecordTime = ros::Time::now();
        return;
    }

    // LiDAR
    if(sMsgFlag[1] == '1')
    {
        std::unique_lock<std::mutex> lock1(_mtxCallbackLiDAR);
        if(!_mqLiDARBuf.empty())
        {
            auto dataLiDAR = _mqLiDARBuf.front();
            if(_mrtStartRecordTime.toSec() < dataLiDAR->header.stamp.toSec())
            {
                _mptrRosbag->write(LIDAR_TOPIC, dataLiDAR->header.stamp, dataLiDAR);
            }
            _mqLiDARBuf.pop();
        }
    }

    // Img Raw
    if(sMsgFlag[2] == '1' && !_mbImgIsCompressed)
    {
        std::unique_lock<std::mutex> lock2(_mtxCallbackImg);
        if(!_mqImgBuf.empty())
        {
            auto dataImg = _mqImgBuf.front();
            if(_mrtStartRecordTime.toSec() < dataImg->header.stamp.toSec())
            {
                _mptrRosbag->write(CAM_TOPIC, dataImg->header.stamp, dataImg);
            }
            _mqImgBuf.pop();
        }
    }

    // Img Compressed
    if(sMsgFlag[2] == '1' && _mbImgIsCompressed)
    {
        std::unique_lock<std::mutex> lock3(_mtxCallbackCImg);
        if(!_mqCImgBuf.empty())
        {
            auto dataCImg = _mqCImgBuf.front();
            if(_mrtStartRecordTime.toSec() < dataCImg->header.stamp.toSec())
            {
                _mptrRosbag->write(CAM_TOPIC, dataCImg->header.stamp, dataCImg);
            }
            _mqCImgBuf.pop();
        }
    }

    // IMU
    if(sMsgFlag[0] == '1')
    {
        std::unique_lock<std::mutex> lock0(_mtxCallbackIMU);
        if(!_mqIMUBuf.empty())
        {
            auto dataImu = _mqIMUBuf.front();
            if(_mrtStartRecordTime.toSec() < dataImu->header.stamp.toSec())
            {
                _mptrRosbag->write(IMU_TOPIC, dataImu->header.stamp, dataImu);
            }
            _mqIMUBuf.pop();
        }
    }
    // IMU2
    if(sMsgFlag[0] == '1' && _mbMultiIMU)
    {
        std::unique_lock<std::mutex> lock0(_mtxCallbackIMU2);
        if(!_mqIMU2Buf.empty())
        {
            auto dataImu = _mqIMU2Buf.front();
            if(_mrtStartRecordTime.toSec() < dataImu->header.stamp.toSec())
            {
                _mptrRosbag->write(IMU_TOPIC_2, dataImu->header.stamp, dataImu);
            }
            _mqIMU2Buf.pop();
        }
    }
}
void Handler::saveData(void)
{
    std::unique_lock<std::mutex> lock0(_mtxCallbackIMU);
    std::unique_lock<std::mutex> lock1(_mtxCallbackLiDAR);
    std::unique_lock<std::mutex> lock2(_mtxCallbackImg);

    while(!_mqLiDARBuf.empty())
    {
        auto dataLiDAR = _mqLiDARBuf.front();
        if(_mrtEndRecordTime.toSec() > dataLiDAR->header.stamp.toSec() && _msRecordFlag__[1] == '1')
        {
            _mptrRosbag->write(LIDAR_TOPIC, dataLiDAR->header.stamp, dataLiDAR);
        }
        _mqLiDARBuf.pop();
    }
    while(!_mqImgBuf.empty())
    {
        auto dataImg = _mqImgBuf.front();
        if(_mrtEndRecordTime.toSec() > dataImg->header.stamp.toSec() && _msRecordFlag__[2] == '1' && !_mbImgIsCompressed)
        {
            _mptrRosbag->write(CAM_TOPIC, dataImg->header.stamp, dataImg);
        }
        _mqImgBuf.pop();
    }
    while(!_mqCImgBuf.empty())
    {
        auto dataCImg = _mqCImgBuf.front();
        if(_mrtEndRecordTime.toSec() > dataCImg->header.stamp.toSec() && _msRecordFlag__[2] == '1' && _mbImgIsCompressed)
        {
            _mptrRosbag->write(CAM_TOPIC, dataCImg->header.stamp, dataCImg);
        }
        _mqCImgBuf.pop();
    }
    while(!_mqIMUBuf.empty())
    {
        auto dataImu = _mqIMUBuf.front();
        if(_mrtEndRecordTime.toSec() > dataImu->header.stamp.toSec() && _msRecordFlag__[0] == '1')
        {
            _mptrRosbag->write(IMU_TOPIC, dataImu->header.stamp, dataImu);
        }
         _mqIMUBuf.pop();
        
    }
    while(!_mqIMU2Buf.empty())
    {
        if(!_mbMultiIMU)break;
        auto dataImu = _mqIMU2Buf.front();
        if(_mrtEndRecordTime.toSec() > dataImu->header.stamp.toSec() && _msRecordFlag__[0] == '1')
        {
            _mptrRosbag->write(IMU_TOPIC_2, dataImu->header.stamp, dataImu);
        }
         _mqIMU2Buf.pop();
        
    }

    _mbRecordFlag = false;
    _mbSaveFlag = false;
    _mptrRosbag->close();
    if(_mptrRosbag != nullptr)delete _mptrRosbag;
}