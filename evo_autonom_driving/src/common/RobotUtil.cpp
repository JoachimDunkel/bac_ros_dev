#include "evo_autonom_driving/common/RobotUtil.hpp"
#include "evo_autonom_driving/common/SharedBehaviour.hpp"
#include <std_msgs/Int8.h>
#include "evo_autonom_driving/common/Util.hpp"

namespace evo_autonom_driving
{

RobotUtil::RobotUtil(ros::NodeHandle& handle)
    : nodeHandle_(handle)
{
    auto shared = SharedBehaviour(nodeHandle_);
    shared.getParameter(Resources::waitAfterLiftCmd, waitAfterLiftCmd_);

    cmdLiftPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(Resources::cmdLiftTopic, 1);
}

geometry_msgs::Pose RobotUtil::getMeanPose(std::vector<geometry_msgs::Pose> captured_poses) 
{
    geometry_msgs::Pose mean_pose;

    float mean_yaw = 0;
    int num_Poses = captured_poses.size();

    for (int i = 0; i < num_Poses; i++)
    {
        auto pose = captured_poses[i];
        float yaw = tf2::getYaw(pose.orientation);
        mean_yaw += yaw;

        mean_pose.position.y += pose.position.y;
        mean_pose.position.x += pose.position.x;
    }

    mean_pose.position.x /= num_Poses;
    mean_pose.position.y /= num_Poses;
    mean_yaw /= num_Poses;

    mean_pose.orientation = Util::quaternionFromRPY(0,0, mean_yaw);

    return mean_pose;
}

 void RobotUtil::SendLiftUpCommand()
{
    std_msgs::Int8 lift_cmd;
    lift_cmd.data = 127;

    cmdLiftPublisher_.publish(lift_cmd);
    //For now it's implemented as wait later we should read joint states or generally make a 
    //more sophisticated controller that does everything continously
    ros::Duration(waitAfterLiftCmd_).sleep();
}

void RobotUtil::SendLiftDownCommand()
{
    std_msgs::Int8 lift_cmd;
    lift_cmd.data = -127;

    cmdLiftPublisher_.publish(lift_cmd);
    //For now it's implemented as wait later we should read joint states or generally make a 
    //more sophisticated controller that does everything continously
    ros::Duration(waitAfterLiftCmd_).sleep();
}

}

