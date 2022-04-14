#pragma once

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace evo_autonom_driving {
    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<PCLPoint> PCLCloud;

    namespace Resources {

        std::string const useICP = "use_icp";

        std::string const combinedScanTopic =  "topic_combined_scan";
        std::string const filteredPointCloudTopic = "topic_cloud_filtered";
        std::string const environmentCloudTopic = "topic_cloud_environment";
        std::string const tableTemplateCloudCombinedTopic = "topic_table_template_cloud_combined";

        std::string const targetTransformation = "topic_target_tranformation";
        std::string const frameBaseLinkId = "frame_base_link_id";

        std::string const moveBaseSimpleGoal = "/move_base_simple/goal";

        std::string const setFilterServiceName = "set_filter_service";

        std::string const sampledTableTemplate = "sampled_table_template.pcd";

        std::string const transformedCloud = "topic_transformed_cloud";

        std::string const packageName = "evo_autonom_driving";

        std::string const twoLegsDistanceThreshold = "two_legs_distance_threshold";

        constexpr float tableLength = 0.670;
        constexpr float legFrameLength = 0.040;
        constexpr float tableDiagonalLength = 0.94752; //tableLength * sqrt(2)

        constexpr float robotLength = 1.040; //when in lowered position
        constexpr float robotWidth = 0.575;
        constexpr float robotWheelFrontalStickOutDistance = 0.045;

        constexpr float driveInfrontDist = 1;

        std::string const filterMeanK = "filter_mean_k";
        std::string const filterRadiusSearch = "filter_radius_search";
        std::string const filterStdDevThreshold = "filter_stdDev_threshold";
        std::string const filterMinNeighboursInRadius = "filter_min_neighbours_in_radius";

        std::string const odomTopic = "odom";
        std::string const baseLinkTopic = "base_link";
        std::string const mapTopic = "map";

        std::string const cmdLiftTopic = "/cmd_lift";
        std::string const cmdVelTopic = "/cmd_vel";
        std::string const waitAfterLiftCmd = "wait_after_lift_cmd";

        std::string const angleDeviationTreshold = "angle_dev_treshold";
        std::string const parallelDevThreshold = "parallel_dev_threshold";
        std::string const acceptedDistTreshhold = "accepted_distance_treshold";

        std::string const maxAccOutside = "max_acceleration_outside";
        std::string const maxAccInside = "max_acceleration_inside";
        std::string const minYDistanceTableLegs = "min_y_distance_table_legs";
        std::string const blockRotationBelow = "block_rotation_below";
        std::string const onlyDriveForwardBelow = "only_drive_forward_below";
        
        std::string const killDriveInsideNodesCmd = "rosnode kill /evo_autonom_driving/drive_beneath_table_controller /evo_autonom_driving/point_cloud_filter /evo_autonom_driving/target_localizer";
        std::string const drivingService = "driving_service";

        std::string const driveInsideAction = "drive_inside_action";
        std::string const driveInsideActionServer = "drive_inside_action_server";
        std::string const nameSpace = "evo_autonom_driving";

    }
}
