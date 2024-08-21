#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver/CustomMsg.h>

ros::Publisher pubIMU;
ros::Publisher pubLiDAR; 
ros::Publisher pubImage;

void callbackIMU(const sensor_msgs::Imu::ConstPtr &msgIMU)
{
    auto tmpMsg = *msgIMU;
    tmpMsg.header.stamp = ros::Time::now();
    pubIMU.publish(tmpMsg);
}
void callbackLiDAR(const livox_ros_driver::CustomMsg::ConstPtr &msgLiDAR)
{
    auto tmpMsg = *msgLiDAR;
    tmpMsg.header.stamp = ros::Time::now();
    pubLiDAR.publish(tmpMsg);
}
void callbackImage(const sensor_msgs::ImageConstPtr &msgIMG)
{
    auto tmpMsg = *msgIMG;
    tmpMsg.header.stamp = ros::Time::now();
    pubImage.publish(tmpMsg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "handheld_forntend_reindexing");
    ros::NodeHandle nh_("~");

    std::string IMU_TOPIC, LIDAR_TOPIC, CAM_TOPIC;
    nh_.param<std::string>("/sensor/imu_topic", IMU_TOPIC, "/");
    nh_.param<std::string>("/sensor/lidar_topic", LIDAR_TOPIC, "/");
    nh_.param<std::string>("/sensor/cam_topic", CAM_TOPIC, "/");

    pubIMU = nh_.advertise<sensor_msgs::Imu>("/cocel_handheld/imu",1000);
    pubLiDAR = nh_.advertise<livox_ros_driver::CustomMsg>("/cocel_handheld/lidar",1000);
    pubImage = nh_.advertise<sensor_msgs::Image>("/cocel_handheld/camera",1000);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Subscriber subImuRaw = nh_.subscribe(IMU_TOPIC, 1000, callbackIMU);
    ros::Subscriber subImuLiDAR = nh_.subscribe(LIDAR_TOPIC, 1000, callbackLiDAR);
    ros::Subscriber subImuImage = nh_.subscribe(CAM_TOPIC, 1000, callbackImage);

    ros::waitForShutdown();

    return 0;
}
