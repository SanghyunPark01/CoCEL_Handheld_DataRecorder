#include "CoCEL_Handheld_DataRecorder/handler.h"

void callbackShutdown(const std_msgs::Bool &msg)
{
    ros::shutdown();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "handheld_forntend_handler");
    ros::NodeHandle nh_("~");
    ros::Subscriber subShutdown_ = nh_.subscribe("/cocel_handheld/shutdown_flag/8513211",10,callbackShutdown);

    Handler handler(nh_);

    std::thread thread_process(&Handler::processHandler, &handler);

    // ros::AsyncSpinner spinner(6);
    // spinner.start();
    ros::spin();

    signal(SIGINT, mySigintHandler);
    ros::waitForShutdown();

    if(!ros::ok())
    {
        std::cout << "[STATUS]Process shut down!" << "\n";
        // system("rosnode list | grep -v rosout | xargs rosnode kill");
        system("killall -9 rosmaster && killall -9 rosout"); // all program kill
    }

    return 0;
}
