#include <gtest/gtest.h>
#include "evo_autonom_driving/common/Util.hpp"
#include <math.h> 
#include "evo_autonom_driving/InitialEstimateCalculator.hpp"
#include "evo_autonom_driving/common/Resources.hpp"
#include <pcl/common/transforms.h>

using namespace evo_autonom_driving;

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLCloud;


class InitialEstimatorTests : public ::testing::Test
{
protected:
    virtual void SetUp() {
        angle_result = -1;
        x_result = -1;
        y_result = -1;

        exp_angle = 0;
        exp_x = 0;
        exp_y = 0;

        side_length = Resources::tableLength - Resources::legFrameLength;
        initial_estimate.setIdentity();
        sut = InitialEstimateCalculator();

        float start = -side_length/2;
        float next = side_length/2;

        sut.left_lower = PCLPoint(start,next,0);
        sut.left_upper = PCLPoint(next,next,0);
        sut.right_lower = PCLPoint(start,start,0);
        sut.right_upper = PCLPoint(next,start,0);
    }

    virtual void TearDown(){

    }

    InitialEstimateCalculator sut;
    float side_length;
    Eigen::Matrix4f initial_estimate;

    float angle_result;
    float x_result;
    float y_result;

    float exp_angle;
    float exp_x;
    float exp_y;

    void exerciseSut(){
        sut.computeInitialEstimate(initial_estimate);

        x_result = initial_estimate(0,3);
        y_result = initial_estimate(1,3);
        angle_result = atan2(initial_estimate(1,0), initial_estimate(0,0));
    }

    void assertSut(){
        float tolerance = 0.000001;    
        EXPECT_NEAR(exp_angle, angle_result, tolerance);
        EXPECT_NEAR(exp_x, x_result, tolerance);
        EXPECT_NEAR(exp_y, y_result, tolerance);
    }

    void rotatePointsBy(float angle){

        PCLCloud::Ptr source(new PCLCloud);
        source->push_back(sut.left_lower);
        source->push_back(sut.left_upper);
        source->push_back(sut.right_lower);
        source->push_back(sut.right_upper);
        
        PCLCloud::Ptr transformed(new PCLCloud);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        transform.rotate(Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitZ()));

        pcl::transformPointCloud (*source, *transformed, transform);

        sut.left_lower = transformed->at(0);
        sut.left_upper = transformed->at(1);
        sut.right_lower = transformed->at(2);
        sut.right_upper = transformed->at(3);
    }

};

TEST_F(InitialEstimatorTests, basicSquareTest)
{
    float start = 1;
    float next = side_length + start;

    sut.left_lower = PCLPoint(start,next,0);
    sut.left_upper = PCLPoint(next,next,0);
    sut.right_lower = PCLPoint(start,start,0);
    sut.right_upper = PCLPoint(next,start,0);

    exerciseSut();

    exp_angle = 0.0;
    exp_x = 1.315;
    exp_y = 1.315;

    assertSut();
}

TEST_F(InitialEstimatorTests, rotatedSquareTest)
{
    exp_angle = 20 * M_PI / 180;

    rotatePointsBy(exp_angle);

    exerciseSut();

    assertSut();
}

TEST_F(InitialEstimatorTests, negativeRotationTest)
{
    exp_angle = -20 * M_PI / 180;

    rotatePointsBy(exp_angle);

    exerciseSut();

    assertSut();
}

TEST_F(InitialEstimatorTests, threeLegsAvailableTest)
{
    PCLPoint invalid;
    invalid.x = 50000;

    sut.left_lower = invalid;

    exerciseSut();
    assertSut();
}


TEST_F(InitialEstimatorTests, twoLegsAvailable)
{
    PCLPoint invalid;
    invalid.x = 50000;

    sut.left_lower = invalid;
    sut.right_upper = invalid;

    exerciseSut();
    assertSut();
}

//TODO tests with not all 4 legs existing.