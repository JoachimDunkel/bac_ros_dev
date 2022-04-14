#include <ros/ros.h>
#include "evo_autonom_driving/DriveBeneathTableController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_localizer");
    ros::NodeHandle nodeHandle("~");

    evo_autonom_driving::DriveBeneathTableController driveController(nodeHandle);

    ros::spin();
    return 0;
}
