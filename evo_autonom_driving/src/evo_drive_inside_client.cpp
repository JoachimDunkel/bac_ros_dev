#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "evo_autonom_driving/DriveInsideAction.h"
#include "evo_autonom_driving/common/Resources.hpp"


//NOTE: this is an example client to show how integrating this drive_inside system would work 
// into the bigger robot system.

using namespace evo_autonom_driving;

const std::string tag_ = "[DRIVE_INSIDE_CLIENT] ";

int main(int argc, char** argv)                     
{
    std::string ros_server_naming = Resources::driveInsideActionServer + "/" + Resources::driveInsideAction;

    ros::init(argc, argv, "drive_inside_client"); 
    ros::NodeHandle nh("~");
    actionlib::SimpleActionClient<DriveInsideAction> 
        action_client(ros_server_naming, true);

    ROS_INFO("%s Waiting for action server to start.", tag_.c_str());

    action_client.waitForServer();

    ROS_INFO("%s Action-server started sending goal", tag_.c_str());

    DriveInsideGoal goal;
    goal.target_pose.header.stamp = ros::Time(0);

    //The rough table_middle position will have to be computed by a higher-level system and 
    //put inside here as goal.
    goal.target_pose.pose.position.x = 1.5; //this number works with my simulation setup.
    action_client.sendGoal(goal);

    ROS_INFO("%s waiting for server result.", tag_.c_str());

    //wait for the action to return
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(180.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = action_client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else{
        ROS_INFO("Action did not finish before the time out.");
    }
                                   
    return 0;
}
