#pragma once

#include <ros/ros.h>
#include "evo_autonom_driving/common/Resources.hpp"
#include "evo_autonom_driving/common/RobotUtil.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>

namespace evo_autonom_driving {

class DriveBeneathTableController {
    public:
        explicit DriveBeneathTableController(ros::NodeHandle& handle);

        void targetTransformationSubscriptionCallback(const geometry_msgs::PoseStamped::ConstPtr& message);
        void tableCloudSubscriptionCallback(const PCLCloud::ConstPtr& message);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& message);

    private:

        enum State 
        {
            READY = 0,
            LIFT_DOWN = 1, 
            DRIVING_INFRONT = 2, 
            DRIVING_INSIDE = 3, 
            FINISHED = 4
        };

        struct DesiredGoalDistance{
            double x_offset = 0.9;
            double y_offset = 0.005;
            double theta_offset = 0.01;
        };

        struct EvoMotionProperties{
            double max_acceleration_inside = 0.5;
            double max_acceleration_outside = 0.01;
            double min_y_distance_table_legs = 0.005;
            double block_rotation_below = 0.5;
            double only_drive_forward_below = 0.0;
        };

        struct DriveInstructions{
            bool goal_reached = false;
            bool drive_forward = true;
            bool drive_sideways = true;
            bool rotate = true;
        };

        ros::NodeHandle & nodeHandle_;
        std::string const tag_ = "[DRIVE_BENEATH_TABLE_CONTROLLER]";
        RobotUtil robotUtility_;

        ros::Subscriber targetTransformationSubscriber_;
        ros::Subscriber tableCloudSubscriber_;
        ros::Publisher cmdVelPublisher_;
        ros::Subscriber odomSubscriber_;
        ros::ServiceClient drivingServiceClient_;

        State currentState_;
        State previousState_;
        PCLCloud tableCloud_;
        geometry_msgs::Twist currentVelocity_;

        geometry_msgs::Pose navigationGoal_;

        size_t relevantRecentPoses_;

        double acceptedDistTreshhold_;
        DesiredGoalDistance desiredDistance_;
        EvoMotionProperties motionProperties_;

        void ChangeStateTo(State newState, std::string info);

        bool Drive();

        DriveInstructions getDriveInstructions(const geometry_msgs::Twist & twist);
        DriveInstructions captureDesiredMotion(geometry_msgs::Twist & twist);
        void constrainVelocity(geometry_msgs::Twist & twist);
        void constraintToFeasableMotion(geometry_msgs::Twist & twist);

        bool motionFeasable(
            const geometry_msgs::Twist & twist, bool test_x, bool test_y, bool test_yaw);
};

bool Drive();
};