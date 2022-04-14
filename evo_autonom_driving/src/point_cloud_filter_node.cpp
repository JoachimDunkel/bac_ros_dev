#include <ros/ros.h>
#include "evo_autonom_driving/PointCloudFilter.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_filter");
    ros::NodeHandle nodeHandle("~");

    evo_autonom_driving::PointCloudFilter pCloudFilter(nodeHandle);

    ros::spin();
    return 0;
}
