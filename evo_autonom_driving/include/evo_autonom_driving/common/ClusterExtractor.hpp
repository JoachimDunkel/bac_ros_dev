#pragma once

#include <ros/ros.h>
#include <pcl/segmentation/extract_clusters.h>
#include "evo_autonom_driving/common/Resources.hpp"

namespace evo_autonom_driving {
    
class ClusterExtractor{

    public:
        explicit ClusterExtractor(ros::NodeHandle& handle);
        std::vector<PCLCloud::Ptr> extractAllCluster(PCLCloud::Ptr cloud_in);
        std::vector<PCLPoint> extractCentroids(std::vector<PCLCloud::Ptr> clusters);
        void SetCloudPointsFromClusters(PCLCloud::Ptr cloud, std::vector<PCLCloud::Ptr> clusters);
        
    private:
        pcl::EuclideanClusterExtraction<PCLPoint> extractor_;
};
};