#include "evo_autonom_driving/TargetLocalizer.hpp"
#include "evo_autonom_driving/common/SharedBehaviour.hpp"
#include "evo_autonom_driving/InitialEstimateCalculator.hpp"
#include "evo_autonom_driving/common/GICP.hpp"
#include "evo_autonom_driving/common/Util.hpp"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

namespace evo_autonom_driving {


    TargetLocalizer::TargetLocalizer(ros::NodeHandle& handle) 
                : nodeHandle_(handle), clusterExtractor_(ClusterExtractor(nodeHandle_))
    {
        auto shared = SharedBehaviour(nodeHandle_);
        shared.useDataFolder().readCloudFromFile(Resources::sampledTableTemplate, targetCloud4Legs_);

        shared.getParameter(Resources::twoLegsDistanceThreshold, twoLegsDistanceThreshold_);
        shared.getParameter(Resources::useICP, useICP_);
        if(!useICP_){
            ROS_INFO("%s ICP disabled.", tag_.c_str());
        }

        std::string topic_cloud_filtered, topic_target_tranformation, 
            target_transform_topic, topic_table_template_cloud_combined;
        shared.getParameter(Resources::filteredPointCloudTopic, topic_cloud_filtered);
        shared.getParameter(Resources::transformedCloud, topic_target_tranformation);
        shared.getParameter(Resources::targetTransformation, target_transform_topic);
        shared.getParameter(Resources::tableTemplateCloudCombinedTopic, topic_table_template_cloud_combined);


        navGoalPublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>(target_transform_topic, 1);
        filteredCloudSubscriber_ = nodeHandle_.subscribe<PCLCloud>(topic_cloud_filtered, 1, &TargetLocalizer::filteredCloudSubscriptionCallback, this);
        transformedTemplatePublisher_ = nodeHandle_.advertise<PCLCloud>(topic_target_tranformation, 1);
        combinedTableTemplatePublisher_ = nodeHandle_.advertise<PCLCloud>(topic_table_template_cloud_combined, 1);
        ROS_INFO("%s Started successfully.", tag_.c_str());
    }

    void TargetLocalizer::filteredCloudSubscriptionCallback(const PCLCloud::ConstPtr& message)
    {
        PCLCloud::Ptr table_cloud(new PCLCloud);
        *table_cloud = *message;
        
        if(table_cloud->size() == 0){
            return;
        }

        PCLCloud::Ptr icp_result_cloud(new PCLCloud);
        *icp_result_cloud = *message;
        icp_result_cloud->clear();

        PCLCloud::Ptr init_guessed_template_cloud(new PCLCloud);
        *init_guessed_template_cloud = *message;
        init_guessed_template_cloud->clear();

        // === Choose method ===
        PCLCloud::Ptr template_cloud(new PCLCloud);

        //TODO always use 4 legs and strip the unnecessary ones.
        *template_cloud = targetCloud4Legs_;
        template_cloud->header = table_cloud->header;

        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        auto clusters = clusterExtractor_.extractAllCluster(table_cloud);
        createTemplateAndInitialEstimate(clusters, template_cloud, initial_guess);

        pcl::transformPointCloud(*template_cloud, *init_guessed_template_cloud, initial_guess);

        // === Perform ICP ===
        Eigen::Matrix4f full_transformation = initial_guess;

        //only perform icp if there are enough points, so we do not run into a pcl internal error
        if(useICP_ && table_cloud->size() >= 20 ){
            Eigen::Matrix4f transform_icp = GICP::calcICPTransformation(init_guessed_template_cloud, table_cloud, icp_result_cloud);
            full_transformation = transform_icp * initial_guess;
        }

        pcl::transformPointCloud(*template_cloud, *template_cloud, full_transformation);
        transformedTemplatePublisher_.publish(template_cloud);

        PCLCloud::Ptr combined_table_template_cloud(new PCLCloud);
        *combined_table_template_cloud = targetCloud4Legs_;
        pcl::transformPointCloud(*combined_table_template_cloud, *combined_table_template_cloud, full_transformation);
        *combined_table_template_cloud += *table_cloud;
        combined_table_template_cloud->header = message->header;
        combinedTableTemplatePublisher_.publish(combined_table_template_cloud);
        
        geometry_msgs::PoseStamped target_pose;
        target_pose.pose = Util::extractPoseFrom(full_transformation);
        pcl_conversions::fromPCL(message->header, target_pose.header);
        navGoalPublisher_.publish(target_pose);

        lastTargetGoal_ = target_pose.pose;
    }

    void TargetLocalizer::createTemplateAndInitialEstimate( std::vector<PCLCloud::Ptr> clusters,
                          PCLCloud::Ptr template_cloud, Eigen::Matrix4f & initial_estimate) {
        static bool first_call = true;

        auto centroids = clusterExtractor_.extractCentroids(clusters);
        if(centroids.size() > 4){
            ROS_ERROR("Number of found clusters on table laser data - exceeds 4.\n This is probably because there are unwanted objects in the area around the table.");
            ros::requestShutdown();
        }

        struct closer_in_x{
            bool operator() (const PCLPoint& a, const PCLPoint& b) const
            {
                return a.x < b.x; 
            }
        };

        struct left_leg_first{
            bool operator() (const PCLPoint& a, const PCLPoint& b) const
            { //the left leg is the one with the bigger y value.
                return a.y > b.y;
            }
        };

        auto estimator = InitialEstimateCalculator();

        PCLCloud::Ptr result_template(new PCLCloud);

        std::sort(centroids.begin(), centroids.end(), closer_in_x());

        if(centroids.size() == 4){
            std::vector<PCLPoint> lower_legs = {centroids[0], centroids[1]};

            std::sort(lower_legs.begin(), lower_legs.end(), left_leg_first());

            std::vector<PCLPoint> upper_legs = {centroids[2], centroids[3]};
            std::sort(upper_legs.begin(), upper_legs.end(), left_leg_first());

            estimator.left_upper = upper_legs[0];
            estimator.right_upper = upper_legs[1];
            estimator.left_lower = lower_legs[0];
            estimator.right_lower = lower_legs[1];

            estimator.computeInitialEstimate(initial_estimate);
            return;
        }

        //sampled_table_template is made out of 64 points 12 points per leg.
        //the legs are orderd in clock-wise orientation starting with the lower-left one
        auto template_legs = clusterExtractor_.extractAllCluster(template_cloud);
        template_cloud->clear();

        if(centroids.size() == 2){

            std::vector<PCLPoint> legs = {centroids[0], centroids[1]};
            std::sort(legs.begin(), legs.end(), left_leg_first());

            auto closest_p = centroids[0];
            //we are to far away from the table we can only have the lower legs case
            if(closest_p.x >= twoLegsDistanceThreshold_){ 
                estimator.left_lower = legs[0];
                estimator.right_lower = legs[1];
            }
            else{
                //bool diagonal_case, lower_legs_case, right_legs_case;
                double leg_dist_thresh = Resources::tableLength / 2;

                //ether left_legs or right_legs
                if(legs[0].y - legs[1].y < leg_dist_thresh){
                    
                    //left_legs
                    if(legs[0].y > 0){
                        if(legs[0].x < legs[1].x){
                            estimator.left_lower = legs[0];
                            estimator.left_upper = legs[1];
                        }
                        else{
                            estimator.left_upper = legs[0];
                            estimator.left_lower = legs[1];
                        }
                    }
                    //right_legs
                    else{
                        if(legs[0].x < legs[1].x){
                            estimator.right_lower = legs[0];
                            estimator.right_upper = legs[1];
                        }
                        else{
                            estimator.right_upper = legs[0];
                            estimator.right_lower = legs[1];
                        }
                    }
                }
                //ether lower or upper legs
                else if(legs[0].x - legs[1].x < leg_dist_thresh){
                    
                    //this should hold if we do not flicker extremely
                    if(first_call || closest_p.x > lastTargetGoal_.position.x + Resources::tableLength / 8 ){
                        estimator.left_upper = legs[0];
                        estimator.right_upper = legs[1];
                        //we can regenerate from falsly setting to upper but not the other way around.
                    }
                    else{
                        estimator.left_lower = legs[0];
                        estimator.right_lower = legs[1];
                    }
                }
                else { //assumption getting left upper and right_lower diagonal case is not possible
                    estimator.left_lower = legs[0];
                    estimator.right_upper = legs[1];
                }
            }
        }
        else{ // 3 leg case.
            //The smallest x-val is one of the lower legs,
            //The highest is on of the upper-legs,
            //The one inbetween is upper or lower depending on which other leg it is closer to in x
            //Again this constraint only works if we are not to far away form the table in an angle not more the 40°
            std::vector<PCLPoint> lower_legs;
            std::vector<PCLPoint> upper_legs;

            lower_legs.push_back(centroids[0]);
            upper_legs.push_back(centroids[2]);

            float dist_to_lower = abs(centroids[1].x - centroids[0].x);
            float dist_to_upper = abs(centroids[1].x - centroids[2].x);

            //TODO extract method and reuse for both.
            if(dist_to_lower < dist_to_upper){
                lower_legs.push_back(centroids[1]);

                //Decide which one the upper legs is based on it's y distance to the other ones
                std::sort(lower_legs.begin(), lower_legs.end(), left_leg_first());
                estimator.left_lower = lower_legs[0];
                estimator.right_lower = lower_legs[1];

                float dist_to_left = abs(centroids[2].y - lower_legs[0].y);
                float dist_to_right = abs(centroids[2].y - lower_legs[1].y);

                if(dist_to_left < dist_to_right){
                    estimator.left_upper = centroids[2];
                }
                else{
                    estimator.right_upper = centroids[2];
                }
            }
            else{
                upper_legs.push_back(centroids[1]);

                //Decide which one the lower legs is based on it's y distance to the other ones
                std::sort(upper_legs.begin(), upper_legs.end(), left_leg_first());
                estimator.left_upper = upper_legs[0];
                estimator.right_upper = upper_legs[1];

                float dist_to_left = abs(centroids[0].y - upper_legs[0].y);
                float dist_to_right = abs(centroids[0].y - upper_legs[1].y);

                if(dist_to_left < dist_to_right){
                    estimator.left_lower = centroids[0];
                }
                else{
                    estimator.right_lower = centroids[0];
                }
            }
        }

        if(estimator.isSet(estimator.left_lower)){
            *template_cloud += *(template_legs[0]);
        }
        if(estimator.isSet(estimator.left_upper)){
            *template_cloud += *(template_legs[1]);
        }
        if(estimator.isSet(estimator.right_upper)){
            *template_cloud += *(template_legs[2]);
        }
        if(estimator.isSet(estimator.right_lower)){
            *template_cloud += *(template_legs[3]);
        }

        estimator.computeInitialEstimate(initial_estimate);

        first_call = false;
    }
}