#include <std_msgs/Int8.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <boost/math/special_functions/sign.hpp>

#include "evo_autonom_driving/DriveBeneathTableController.hpp"
#include "evo_autonom_driving/common/SharedBehaviour.hpp"
#include "evo_autonom_driving/common/Util.hpp"
#include "evo_autonom_driving/common/GICP.hpp"
#include <std_srvs/SetBool.h>

namespace evo_autonom_driving {

    DriveBeneathTableController::DriveBeneathTableController(ros::NodeHandle& handle) : nodeHandle_(handle), robotUtility_(handle)
    {
        auto shared = SharedBehaviour(nodeHandle_);

        shared.getParameter(Resources::angleDeviationTreshold, desiredDistance_.theta_offset);
        shared.getParameter(Resources::parallelDevThreshold, desiredDistance_.y_offset);
        shared.getParameter(Resources::acceptedDistTreshhold, acceptedDistTreshhold_);
        desiredDistance_.x_offset = Resources::driveInfrontDist;

        shared.getParameter(Resources::maxAccInside, motionProperties_.max_acceleration_inside);
        shared.getParameter(Resources::maxAccOutside, motionProperties_.max_acceleration_outside);
        shared.getParameter(Resources::minYDistanceTableLegs, motionProperties_.min_y_distance_table_legs);
        shared.getParameter(Resources::blockRotationBelow, motionProperties_.block_rotation_below);
        shared.getParameter(Resources::onlyDriveForwardBelow, motionProperties_.only_drive_forward_below);

        std::string topic_table_template_cloud_combined, topic_target_tranformation;
        shared.getParameter(Resources::tableTemplateCloudCombinedTopic, topic_table_template_cloud_combined);
        shared.getParameter(Resources::targetTransformation, topic_target_tranformation);

        targetTransformationSubscriber_ = nodeHandle_.subscribe<geometry_msgs::PoseStamped>
                (topic_target_tranformation, 1, &DriveBeneathTableController::targetTransformationSubscriptionCallback, this);

        tableCloudSubscriber_ = nodeHandle_.subscribe<PCLCloud>
            (topic_table_template_cloud_combined, 1, &DriveBeneathTableController::tableCloudSubscriptionCallback, this);
        cmdVelPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(Resources::cmdVelTopic, 1);
        odomSubscriber_ = nodeHandle_.subscribe<nav_msgs::Odometry>(Resources::odomTopic, 1, &DriveBeneathTableController::odomCallback, this);

        std::string driving_service_naming = "/" + Resources::nameSpace + "/" + Resources::driveInsideActionServer + "/" + Resources::drivingService;

        drivingServiceClient_ = nodeHandle_.serviceClient<std_srvs::SetBool>(driving_service_naming);

        currentState_ = State::READY;

        ROS_INFO("%s Started succesfully", tag_.c_str());
    }

    void DriveBeneathTableController::targetTransformationSubscriptionCallback(const geometry_msgs::PoseStamped::ConstPtr& message) 
    {
        navigationGoal_ = message->pose;
                switch (currentState_)
        {            
            case State::READY:
            {
                //Later we should wait for a signal to start driving or something.
                ros::Duration(1).sleep();
                ChangeStateTo(State::LIFT_DOWN, "Starting lift-down");

                return;
            }
            case State::LIFT_DOWN:
            {
                robotUtility_.SendLiftDownCommand();
                ChangeStateTo(State::DRIVING_INFRONT, "Positioning infront the table");

                return;
            }
            case State::DRIVING_INFRONT:
            {
                desiredDistance_.x_offset = Resources::driveInfrontDist;
                if(Drive()){
                    ChangeStateTo(State::DRIVING_INSIDE, "Driving inside");
                }
                return;
            }
            case State::DRIVING_INSIDE:
            {    
                desiredDistance_.x_offset = acceptedDistTreshhold_;           
                if(Drive()){
                    ChangeStateTo(State::FINISHED, "Finished");
                }
                return;
            }            
            case State::FINISHED:
            {
                std_srvs::SetBool srv;
                srv.request.data = true;
                if(drivingServiceClient_.call(srv))
                {
                    ROS_INFO("%s signaled driving inside finished.", tag_.c_str());
                    ros::requestShutdown();
                }
                else
                {
                    ROS_ERROR("%s failed to signal driving finished. Retrying", tag_.c_str());
                }
                return;
            }
            default:
            {
                ROS_ERROR("%s Unhandeled state. Shutting down.", tag_.c_str());
                ros::requestShutdown();
                return;
            }
        }
    }

    void DriveBeneathTableController::tableCloudSubscriptionCallback(const PCLCloud::ConstPtr& message) 
    {
        tableCloud_ = *message;
    }

    void DriveBeneathTableController::odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
        currentVelocity_ = message->twist.twist;
    }

    void DriveBeneathTableController::ChangeStateTo(State newState, std::string info){
        ROS_INFO("%s %s", tag_.c_str(), info.c_str());
        previousState_ = currentState_;
        currentState_ = newState;

    }


    bool DriveBeneathTableController::Drive()           
    {
        geometry_msgs::Twist twist = geometry_msgs::Twist();
        
        auto instructions = captureDesiredMotion(twist);
        if(instructions.goal_reached) return true;
 
        constrainVelocity(twist);

        constraintToFeasableMotion(twist);

        cmdVelPublisher_.publish(twist);
        return false;
    }


    DriveBeneathTableController::DriveInstructions DriveBeneathTableController::getDriveInstructions(const geometry_msgs::Twist & twist){

        DriveInstructions instructions;
        if(fabs(twist.angular.z) <= desiredDistance_.theta_offset){
            instructions.rotate = false;
        }
        if(fabs(twist.linear.y) <= desiredDistance_.y_offset){
            instructions.drive_sideways = false;
        }
        if(fabs(twist.linear.x) <= desiredDistance_.x_offset){
            instructions.drive_forward = false;
        }
        instructions.goal_reached = !instructions.drive_forward;

        if(currentState_ == State::DRIVING_INFRONT)
        {
            instructions.goal_reached = (!instructions.drive_forward 
                                        && !instructions.drive_sideways
                                        && !instructions.rotate);
        }

        return instructions;    
    }

    DriveBeneathTableController::DriveInstructions 
            DriveBeneathTableController::captureDesiredMotion(geometry_msgs::Twist & twist) 
    {
        float yaw_offset = tf2::getYaw(navigationGoal_.orientation);
        twist.angular.z = yaw_offset;

        float y_offset = navigationGoal_.position.y;
        twist.linear.y = y_offset;
    
        float x_offset = navigationGoal_.position.x;
        twist.linear.x = x_offset;

        auto instructions = getDriveInstructions(twist);

        if(!instructions.drive_forward){
            twist.linear.x = 0;
        }
        if(!instructions.drive_sideways){
            twist.linear.y = 0;
        }
        if(!instructions.rotate){
            twist.angular.z = 0;
        }

        return instructions;
    }

    void DriveBeneathTableController::constrainVelocity(geometry_msgs::Twist & twist) 
    {
        double max_acceleration = motionProperties_.max_acceleration_outside;
        if(currentState_ == State::DRIVING_INSIDE){
            max_acceleration = motionProperties_.max_acceleration_inside;
        }
        twist.linear.x = Util::ClipOfValueAt(twist.linear.x, max_acceleration);
        twist.linear.y = Util::ClipOfValueAt(twist.linear.y, max_acceleration);
        twist.angular.z = Util::ClipOfValueAt(twist.angular.z, max_acceleration);
    }

    //TODO do not look at cases irrelevant by drive instructions
    //TODO refactor this. the 3 bools are unnecessary. also check for goal_reached and so on before this method
    void DriveBeneathTableController::constraintToFeasableMotion(geometry_msgs::Twist & twist) 
    {
        auto instructions = getDriveInstructions(twist);
        if(instructions.goal_reached){
            return;
        }

        if(navigationGoal_.position.x < motionProperties_.block_rotation_below){
            twist.angular.z = 0.0;
        }

        if(navigationGoal_.position.x < motionProperties_.only_drive_forward_below){
            twist.angular.z = 0.0;
            twist.linear.y = 0.0;
            twist.linear.x = motionProperties_.max_acceleration_inside;
            return;
        }

        if(motionFeasable(twist, true, true, true)){ //Try whole motion
            return;
        }
        else if(motionFeasable(twist, true, true, false)){ //Try without rotation
            twist.angular.z = 0.0;
            return;
        }
        else if(motionFeasable(twist, true, false, false)){ //Try x only
            twist.linear.y = 0.0;
            twist.angular.z = 0.0;
            return;
        }
        else if(motionFeasable(twist, false, true, false)){ //Try y only
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            return;
        }
        else if(motionFeasable(twist, false, true, true)){  //Try without x
            twist.linear.x = 0.0;
            return;
        }
        else if(motionFeasable(twist, true, false, true)){ //Try without y
            twist.linear.y = 0.0;
            return;
        }
        else if(motionFeasable(twist, false, false, true)){ //Try rotation only
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            return;
        }
        else{
            twist.linear.x = twist.linear.x /2;
            twist.linear.y = twist.linear.y /2;
            twist.angular.z = twist.angular.z /2;
            constraintToFeasableMotion(twist);
        }
    }

    bool DriveBeneathTableController::motionFeasable(const geometry_msgs::Twist & twist,
                                          bool test_x, bool test_y, bool test_yaw) 
    {
        auto robot_width = Resources::robotWidth / 2 + motionProperties_.min_y_distance_table_legs;
        auto robot_length = Resources::robotLength / 2 + Resources::robotWheelFrontalStickOutDistance + 0.01;
        pcl::CropBox<PCLPoint> bounding_box;
        bounding_box.setMin(Eigen::Vector4f(-robot_length, -robot_width, -1, 1.0));
        bounding_box.setMax(Eigen::Vector4f(robot_length, robot_width, 1, 1.0));

        Eigen::Vector3f translation;

        geometry_msgs::Twist desired_motion;
        desired_motion.linear.x = twist.linear.x + currentVelocity_.linear.x;
        desired_motion.linear.y = twist.linear.y + currentVelocity_.linear.y;
        desired_motion.angular.z = twist.angular.z + currentVelocity_.angular.z;

        if(test_x){
            translation(0) = desired_motion.linear.x;
        }
        if(test_y){
            translation(1) = desired_motion.linear.y;
            //this will work but also probably cause the robot when it not sees a leg to crash into it
        }
        if(test_yaw){
            bounding_box.setRotation(Eigen::Vector3f (0.0f, 0.0f, desired_motion.angular.z));
        }

        bounding_box.setTranslation(translation);

        PCLCloud::Ptr input_cloud(new PCLCloud);
        *input_cloud = tableCloud_;
        bounding_box.setInputCloud(input_cloud);

        PCLCloud::Ptr points_hit(new PCLCloud);
        bounding_box.filter(*points_hit);
        if(points_hit->size() > 0){
            return false;
        }
        return true;
    }
}



