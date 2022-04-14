#include <ros/ros.h>                              
#include <actionlib/server/simple_action_server.h>
#include <cstdlib>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <evo_autonom_driving/DriveInsideAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include "evo_autonom_driving/common/Resources.hpp"
#include "evo_autonom_driving/common/SharedBehaviour.hpp"

namespace evo_autonom_driving
{

class DriveInsideActionServer
{ 
  public:
    DriveInsideActionServer(ros::NodeHandle& handle, std::string name) :
        nh_(handle),
        actionServer_(nh_, name, boost::bind(&DriveInsideActionServer::executeCB, this, _1), false),
        action_name_(name),
        tag_("[DRIVE_INSIDE_ACTION_SERVER] ")
    {
      auto shared = SharedBehaviour(nh_);
      std::string target_transform_topic;
      shared.getParameter(Resources::targetTransformation, target_transform_topic);

      targetTransformationSubscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>
                (target_transform_topic, 1, &DriveInsideActionServer::targetTransformationSubscriptionCallback, this);
      initialTransformationPublisher_ =  nh_.advertise<geometry_msgs::PoseStamped>(target_transform_topic, 1, true);

      drivingService_ = nh_.advertiseService(Resources::drivingService, &DriveInsideActionServer::drivingServiceCallback, this);
      actionServer_.start();
      ROS_INFO("%s started succesfully, awaiting clients.", tag_.c_str());
    }

    void targetTransformationSubscriptionCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
    {
      current_pose = *message;
    }

    bool drivingServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
      if(req.data)
      {
        finishedDriving_ = true;
      }
      res.success = true;
      return true;
    }


    void executeCB(const DriveInsideGoalConstPtr &goal)
    {
      ros::Rate r(1);
      finishedDriving_ = false;
      ros::WallTime start_, end_;
      start_ = ros::WallTime::now();

      boost::thread run_nodes_thread(boost::bind(&DriveInsideActionServer::startNodes, this));

      ROS_INFO("%s Started driving inside.", tag_.c_str());

      initialTransformationPublisher_.publish(goal->target_pose);

      while(true)
      {
        if(actionServer_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s prempted action.", tag_.c_str());
          actionServer_.setPreempted();
          break;
        }

        DriveInsideFeedback feedback;
        feedback.current_pose = current_pose;
        actionServer_.publishFeedback(feedback);

        if(finishedDriving_)
        {
          ROS_INFO("%s finished driving inside",tag_.c_str());
          break;
        }

        r.sleep();
      }

      end_ = ros::WallTime::now();

      if(finishedDriving_)
      {
        DriveInsideResult result;
        result.needed_time.data = (end_ - start_).toSec();
        actionServer_.setSucceeded(result);
      }
    }

  private:      

    void startNodes()
    {
      //Maybe we want to have a more sophisticated approach 
      //that involves monitoring single running nodes and respawning them.
      //take a look at: https://github.com/MisoRobotics/node_director
      //TODO start for real robot dependent of config or something.
      std::system("roslaunch evo_autonom_driving for_simulator.launch");
    }

    ros::NodeHandle & nh_;
    actionlib::SimpleActionServer<DriveInsideAction> actionServer_;
    const std::string action_name_;
    const std::string tag_;
    geometry_msgs::PoseStamped current_pose;
    ros::Subscriber targetTransformationSubscriber_;
    ros::Publisher initialTransformationPublisher_;
    bool finishedDriving_;
    ros::ServiceServer drivingService_;

};
}



int main(int argc, char** argv)                     
{
  std::string name = evo_autonom_driving::Resources::driveInsideAction;
  ros::init(argc, argv, name); 

  ros::NodeHandle nh("~");
  evo_autonom_driving::DriveInsideActionServer action_server(nh, name);
  ros::spin();                                      
  return 0;
}
