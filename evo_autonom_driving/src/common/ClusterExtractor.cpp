#include "evo_autonom_driving/common/ClusterExtractor.hpp"
#include "evo_autonom_driving/common/SharedBehaviour.hpp"

#include <pcl/common/centroid.h>

namespace evo_autonom_driving {

ClusterExtractor::ClusterExtractor(ros::NodeHandle& handle)
{
    extractor_ = pcl::EuclideanClusterExtraction<PCLPoint>();
    float cluster_tolerance;
    int min_neighbours_in_cluster;
    auto shared = SharedBehaviour(handle);
    shared.getParameter(Resources::filterMinNeighboursInRadius, min_neighbours_in_cluster);
    shared.getParameter(Resources::filterRadiusSearch, cluster_tolerance);
    extractor_.setClusterTolerance (cluster_tolerance);
    extractor_.setMinClusterSize(min_neighbours_in_cluster);
}

std::vector<PCLCloud::Ptr> ClusterExtractor::extractAllCluster(PCLCloud::Ptr cloud_in) 
{
    pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint>);
    tree->setInputCloud(cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;

    extractor_.setSearchMethod (tree);
    extractor_.setInputCloud (cloud_in);
    extractor_.extract (cluster_indices);

    std::vector<PCLCloud::Ptr> clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PCLCloud::Ptr cloud_cluster(new PCLCloud);

        for(const auto& idx : it->indices)
        {
            cloud_cluster->push_back((*cloud_in)[idx]);
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }

        clusters.push_back(cloud_cluster);
    }

    return clusters;
}

std::vector<PCLPoint> ClusterExtractor::extractCentroids(std::vector<PCLCloud::Ptr> clusters) 
{
    std::vector<PCLPoint> centroids;

    for(const auto& cluster : clusters){
        PCLPoint centroid;
        pcl::computeCentroid(*cluster, centroid);
        centroids.push_back(centroid);
    }
    return centroids;
}

void ClusterExtractor::SetCloudPointsFromClusters(PCLCloud::Ptr cloud, std::vector<PCLCloud::Ptr> clusters){
    cloud->points.clear();
    cloud->width = 0;
    cloud->height = 1;

    for(const auto cluster : clusters){
        for(const auto point : cluster->points){
            cloud->points.push_back(point);
            cloud->width += 1;
        }
    }
}

};