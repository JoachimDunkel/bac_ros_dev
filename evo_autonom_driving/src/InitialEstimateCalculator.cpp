#include "evo_autonom_driving/InitialEstimateCalculator.hpp"
#include "evo_autonom_driving/common/Util.hpp"

namespace evo_autonom_driving {

InitialEstimateCalculator::InitialEstimateCalculator()
{
    PCLPoint invalid;
    invalid.x = 50000;

    left_lower = invalid;
    left_upper = invalid;
    right_lower = invalid;
    right_upper = invalid;

    num_connections = 0;
    sum_angle = 0.0;
    sum_x = 0.0;
    sum_y = 0.0;
}

bool InitialEstimateCalculator::isSet(PCLPoint table_leg) {
    return table_leg.x < 40000;
}

void InitialEstimateCalculator::computeInitialEstimate(Eigen::Matrix4f & initial_estimate) 
{
    float table_side_to_middle_dist = (Resources::tableLength - Resources::legFrameLength) / 2;

    addAngleAndPositionIfPossible(right_lower,left_upper, 0.0,  M_PI_4);
    addAngleAndPositionIfPossible(right_upper, left_lower,0.0, - M_PI_4);

    addAngleAndPositionIfPossible(left_upper, left_lower, -table_side_to_middle_dist, -M_PI_2);
    addAngleAndPositionIfPossible(right_upper, left_upper , -table_side_to_middle_dist, 0.0);

    addAngleAndPositionIfPossible( right_upper, right_lower,  table_side_to_middle_dist, -M_PI_2);
    addAngleAndPositionIfPossible(right_lower, left_lower,  table_side_to_middle_dist, 0.0);

    float angle = sum_angle / num_connections;
    float x_mean = sum_x / num_connections;
    float y_mean = sum_y / num_connections;

    initial_estimate(1,0) = sin(angle);
    initial_estimate(0,0) = cos(angle);
    initial_estimate(0,1) = -sin(angle);
    initial_estimate(1,1) = cos(angle);

    initial_estimate(0,3) = x_mean;
    initial_estimate(1,3) = y_mean;
}

void InitialEstimateCalculator::addAngleAndPositionIfPossible(
    PCLPoint a, PCLPoint b, float dist_offset, float angle_offset) 
{
    if(!isSet(a) || !isSet(b)){
        return;
    }
    num_connections += 1;

    float angle_between = Util::getAngleBetween(a, b);
    float angle = angle_between + angle_offset;
    sum_angle += angle;

    auto y = (a.y + b.y)/2;
    auto x = (a.x + b.x)/2;

    x += cos(angle_between) * dist_offset;
    y += sin(angle_between) * dist_offset;

    sum_x += x;
    sum_y += y;
}
}



