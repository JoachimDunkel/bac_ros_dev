#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

#include "evo_autonom_driving/common/ClusterExtractor.hpp"

namespace evo_autonom_driving {

class TargetLocalizer{
    public:
        explicit TargetLocalizer(ros::NodeHandle& handle);

        void filteredCloudSubscriptionCallback(const PCLCloud::ConstPtr& message);

    private:

        void createTemplateAndInitialEstimate(
            std::vector<PCLCloud::Ptr> clusters, PCLCloud::Ptr template_cloud, Eigen::Matrix4f & initial_estimate);

        double getInitialAngleEstimate(std::vector<PCLCloud::Ptr> clusters);

        std::string const tag_ = "[TARGET-LOCALIZER]";

        ros::NodeHandle & nodeHandle_;

        PCLCloud targetCloud4Legs_;
        double twoLegsDistanceThreshold_;
        ClusterExtractor clusterExtractor_;
        bool useICP_;

        geometry_msgs::Pose lastTargetGoal_;

        ros::Publisher navGoalPublisher_;
        ros::Subscriber filteredCloudSubscriber_;
        ros::Publisher transformedTemplatePublisher_;
        ros::Publisher combinedTableTemplatePublisher_;

};
}

