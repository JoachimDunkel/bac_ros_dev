#pragma once

#include <ros/ros.h>
#include "evo_autonom_driving/common/Resources.hpp"

namespace evo_autonom_driving {

class InitialEstimateCalculator {

    public:
        explicit InitialEstimateCalculator();

        PCLPoint left_lower;
        PCLPoint left_upper;
        PCLPoint right_lower;
        PCLPoint right_upper;

        bool isSet(PCLPoint table_leg);

        void computeInitialEstimate(Eigen::Matrix4f & initial_estimate);

    private:

        u_int8_t num_connections;
        float sum_angle;
        float sum_x;
        float sum_y;

        void addAngleAndPositionIfPossible(PCLPoint a, PCLPoint b, float dist_offset, float angle_offset);
};
}