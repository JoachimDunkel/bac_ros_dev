#include "evo_autonom_driving/PointCloudFilter.hpp"
#include "evo_autonom_driving/common/SharedBehaviour.hpp"
#include "evo_autonom_driving/common/Util.hpp"

#include <pcl/ml/kmeans.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/distances.h>

namespace evo_autonom_driving {

    PointCloudFilter::PointCloudFilter(ros::NodeHandle& handle) : nodeHandle_(handle), clusterExtractor_(ClusterExtractor(handle))
    {
        tablePositionReceived_ = false;

        auto shared = SharedBehaviour(nodeHandle_);
        shared.getParameter(Resources::filterMeanK, filterParams_.mean_k);
        shared.getParameter(Resources::filterRadiusSearch, filterParams_.radius_search);
        shared.getParameter(Resources::filterStdDevThreshold, filterParams_.stdDev_threshold);
        shared.getParameter(Resources::filterMinNeighboursInRadius, filterParams_.min_neighbours_in_radius_default);
        filterParams_.min_neighbours_in_radius_current = filterParams_.min_neighbours_in_radius_default;

        std::string topic_combined_scan, topic_cloud_filtered, target_transform_topic, topic_cloud_environment;
        shared.getParameter(Resources::combinedScanTopic, topic_combined_scan);
        shared.getParameter(Resources::filteredPointCloudTopic, topic_cloud_filtered);
        shared.getParameter(Resources::targetTransformation, target_transform_topic);
        shared.getParameter(Resources::environmentCloudTopic, topic_cloud_environment);

        mergedCloudsubscriber_ = nodeHandle_.subscribe<PCLCloud>
                (topic_combined_scan, 1, &PointCloudFilter::mergedCloudSubscriptionCallback, this);
        filteredCloudpublisher_ = nodeHandle_.advertise<PCLCloud>(topic_cloud_filtered, 1);
        environmentCloudpublisher_ = nodeHandle_.advertise<PCLCloud>(topic_cloud_environment, 1);

        targetTransformationSubscriber_ = nodeHandle_.subscribe<geometry_msgs::PoseStamped>
                (target_transform_topic, 1, &PointCloudFilter::targetTransformationSubscriptionCallback, this);

        PrepareFilters();

        ROS_INFO("%s started successfully.", tag_.c_str());
    }

    void PointCloudFilter::PrepareFilters()
    {
        statOutRem_.setMeanK(filterParams_.mean_k);
        statOutRem_.setStddevMulThresh(filterParams_.stdDev_threshold);
        radiusOutRem_.setRadiusSearch(filterParams_.radius_search); 
        radiusOutRem_.setMinNeighborsInRadius(filterParams_.min_neighbours_in_radius_current);
    }

    void PointCloudFilter::mergedCloudSubscriptionCallback(const PCLCloud::ConstPtr& message)
    {
        if(!tablePositionReceived_) return;

        PCLCloud::Ptr cloud_in(new PCLCloud);
        *cloud_in = *message;
        cloud_in->is_dense = true;

        statOutRem_.setInputCloud(cloud_in);
        statOutRem_.filter(*cloud_in);
        environmentCloudpublisher_.publish(cloud_in);

        RemoveAllPointsToFarFromTargetPosition(cloud_in);

        radiusOutRem_.setInputCloud(cloud_in);
        radiusOutRem_.filter(*cloud_in);

        if(ExtractTableLegsFrom(cloud_in)){
            filteredCloudpublisher_.publish(cloud_in);
            filterParams_.min_neighbours_in_radius_current = filterParams_.min_neighbours_in_radius_default;
        }
    }

    void PointCloudFilter::RemoveAllPointsToFarFromTargetPosition(PCLCloud::Ptr cloud_in)
    {

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (cloud_in);

        PCLPoint searchPoint; //possible problem: we search in 3d cube and some points are offset to z = 0.49
        searchPoint.x = lastReceivedPosition_.position.x;
        searchPoint.y = lastReceivedPosition_.position.y;
        searchPoint.z = lastReceivedPosition_.position.z;

        float radius = Resources::tableDiagonalLength;

        std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
        std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

        if(kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0){
            ROS_ERROR("%s  Did not find any points within valid distance to target position.", tag_.c_str());
            ros::requestShutdown();
        };

        PCLCloud::Ptr temp_cloud(new PCLCloud);

        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
            temp_cloud->points.push_back(cloud_in->points[ pointIdxRadiusSearch[i]]);
        }

        cloud_in->points.clear();
        cloud_in->height = 1;
        cloud_in->width = pointIdxRadiusSearch.size();
        cloud_in->points = temp_cloud->points;
    }
    
    bool areInValidDistanceToEachOther(const PCLPoint & lhs, const PCLPoint & rhs){
        float distance = pcl::euclideanDistance(lhs, rhs);
        float allowedDeviation = 0.1; //distance between centroids can deviate +- 5cm from the legs centers
        if(distance < Resources::tableDiagonalLength
                && distance > Resources::tableDiagonalLength - allowedDeviation){
            return true;
        }
        else if(distance < Resources::tableLength
                && distance > Resources::tableLength - allowedDeviation){
            return true;
        }
        return false;
    }

    bool PointCloudFilter::ExtractTableLegsFrom(PCLCloud::Ptr cloud_in)
    {
        auto clusters = clusterExtractor_.extractAllCluster(cloud_in);
        if(clusters.size() < 2){
            if(filterParams_.min_neighbours_in_radius_current <= 1){
                filterParams_.min_neighbours_in_radius_current = filterParams_.min_neighbours_in_radius_default;
            }
            filterParams_.min_neighbours_in_radius_current -= 1;
            PrepareFilters();

            return false;
        }

        auto centroids = clusterExtractor_.extractCentroids(clusters);

        PCLPoint target_point;
        target_point.x = lastReceivedPosition_.position.x;
        target_point.y = lastReceivedPosition_.position.y;
        auto sorted_table_candidates = Util::sortByEuclidDistanceTo(centroids, target_point);

        std::vector<PCLPoint> points_in_table;
        points_in_table.push_back(sorted_table_candidates[0]);

        std::queue<PCLPoint> remaining_candidates;
        for (size_t i = 1; i < sorted_table_candidates.size(); i++)
        {
            remaining_candidates.push(sorted_table_candidates[i]);
        }

        while(!remaining_candidates.empty() && points_in_table.size() < 4){
            PCLPoint candidate = remaining_candidates.front();
            remaining_candidates.pop();

            bool isViable = true;
            for(const auto & leg : points_in_table){
                if(!areInValidDistanceToEachOther(leg, candidate)){
                    isViable = false;
                    break;
                }
            }

            if(isViable){
                points_in_table.push_back(candidate);
            }
        }

        //matche welche cluster wirklich genommen wurden und lösche alle anderen punkte.
        //Trivial approach for now (n^2)
        std::vector<PCLCloud::Ptr> matching_clusters;

        for(const auto& table_point : points_in_table){
            int index_of_matching_centroid = -1;

            for (size_t i = 0; i < centroids.size(); i++)
            {
                auto centroid = centroids[i];
                if(table_point.x == centroid.x &&
                    table_point.y == centroid.y)
                {
                    index_of_matching_centroid = i;
                    break;
                }
            }
            if(index_of_matching_centroid == -1){
                ROS_ERROR("%s Recalculating the correct cluster indexes failed for some reason", tag_.c_str());
                return false;
            }

            auto corresponding_cluster = clusters[index_of_matching_centroid];
            matching_clusters.push_back(corresponding_cluster);
        }
        
        clusterExtractor_.SetCloudPointsFromClusters(cloud_in, matching_clusters);
        return true;
    }


    void PointCloudFilter::targetTransformationSubscriptionCallback(const geometry_msgs::PoseStamped::ConstPtr& message) 
    {
        lastReceivedPosition_ = message->pose;

        if(!tablePositionReceived_){
            ROS_INFO("%s Received initial table position.", tag_.c_str());
        }
        tablePositionReceived_ = true;
    }
}