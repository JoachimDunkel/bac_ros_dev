#pragma once

#include "evo_autonom_driving/common/Resources.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>

namespace evo_autonom_driving 
{

class Util 
{
    public:
        static std::vector<PCLPoint> sortByEuclidDistanceTo(std::vector<PCLPoint> points, const PCLPoint& target);

        static void sortByEuclidDistanceTo(PCLCloud::Ptr cloud, const PCLPoint& target);

        static geometry_msgs::Quaternion quaternionFromRPY(float role, float pitch, float yaw);

        static float getAngleBetween(PCLPoint a, PCLPoint b);

        static float ClipOfValueAt(float value, float threshold);
        
        static geometry_msgs::Pose extractPoseFrom(Eigen::Matrix4f transformation);

};
};