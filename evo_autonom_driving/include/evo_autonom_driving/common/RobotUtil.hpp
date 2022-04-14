#pragma once
#include <ros/ros.h>
#include "evo_autonom_driving/common/Resources.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace evo_autonom_driving
{
class RobotUtil
{
    public:
        explicit RobotUtil(ros::NodeHandle& handle);

        geometry_msgs::Pose getMeanPose(std::vector<geometry_msgs::Pose> captured_poses);
        void SendLiftUpCommand();
        void SendLiftDownCommand();

    private:
        ros::NodeHandle & nodeHandle_;
        ros::Publisher cmdLiftPublisher_;
        double waitAfterLiftCmd_;


};
} 
