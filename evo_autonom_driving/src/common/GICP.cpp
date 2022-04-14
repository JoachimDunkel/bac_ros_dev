
#include "evo_autonom_driving/common/GICP.hpp"
#include <pcl/registration/gicp.h>


namespace evo_autonom_driving
{
namespace GICP
{
Eigen::Matrix4f calcICPTransformation(
        PCLCloud::Ptr cloud_in, PCLCloud::Ptr target_cloud, PCLCloud::Ptr result_cloud)
    
{
    pcl::GeneralizedIterativeClosestPoint<PCLPoint, PCLPoint> gicp;
    gicp.setInputSource(cloud_in);
    gicp.setInputTarget(target_cloud);
    gicp.align(*result_cloud);

    return gicp.getFinalTransformation();
}

}
}

