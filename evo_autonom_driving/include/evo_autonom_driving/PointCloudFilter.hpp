#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "evo_autonom_driving/common/ClusterExtractor.hpp"


namespace evo_autonom_driving {

class PointCloudFilter {

    public:
        explicit PointCloudFilter(ros::NodeHandle& handle);

        void mergedCloudSubscriptionCallback(const PCLCloud::ConstPtr& message);
        void targetTransformationSubscriptionCallback(const geometry_msgs::PoseStamped::ConstPtr& message);

    private:

        struct FilterParameters{
            int mean_k = 2;
            int min_neighbours_in_radius_current = 2;
            int min_neighbours_in_radius_default = 2;
            double radius_search = 0.2;
            double stdDev_threshold = 1;
        };

        ros::NodeHandle & nodeHandle_;
        ClusterExtractor clusterExtractor_;
        FilterParameters filterParams_;
        pcl::StatisticalOutlierRemoval<PCLPoint> statOutRem_;
        pcl::RadiusOutlierRemoval<PCLPoint> radiusOutRem_;
        void PrepareFilters();
        void RemoveAllPointsToFarFromTargetPosition(PCLCloud::Ptr cloud_in);
        bool ExtractTableLegsFrom(PCLCloud::Ptr cloud_in);

        bool tablePositionReceived_;
        geometry_msgs::Pose lastReceivedPosition_;


        std::string const tag_ = "[POINT-CLOUD-FILTER]";

        ros::Subscriber targetTransformationSubscriber_;
        ros::Subscriber mergedCloudsubscriber_;
        ros::Publisher filteredCloudpublisher_;
        ros::Publisher environmentCloudpublisher_;

};
}
