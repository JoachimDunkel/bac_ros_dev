#pragma once

#include "evo_autonom_driving/common/Resources.hpp"

namespace evo_autonom_driving
{
namespace GICP
{
    Eigen::Matrix4f calcICPTransformation(
        PCLCloud::Ptr cloud_in, PCLCloud::Ptr target_cloud, PCLCloud::Ptr result_cloud);

} 

}