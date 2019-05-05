#include "TargetGenerator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TargetGenerator");

    ros::NodeHandle nh;
        
    ROS_INFO("main function");
    TargetGenerator tg(&nh);

    ros::spin();
    return 0;
}