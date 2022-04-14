#include <gtest/gtest.h>
#include "evo_autonom_driving/TargetLocalizer.hpp"
#include "evo_autonom_driving/common/Util.hpp"
#include <math.h> 
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace evo_autonom_driving;

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLCloud;


class evo_Tests : public ::testing::Test
{
protected:
    virtual void SetUp() {

    }

    virtual void TearDown(){

    }

};

TEST_F(evo_Tests, sortByEuclidDistWorks)
{
    PCLPoint a, b, c;
    a.x = 1;
    b.x = 2;
    c.x = 3;

    PCLPoint target;
    target.x = 2.6;

    PCLCloud::Ptr cloud(new PCLCloud);
    cloud->points.push_back(a);
    cloud->points.push_back(b);
    cloud->points.push_back(c);

    Util::sortByEuclidDistanceTo(cloud, target);

    EXPECT_EQ(3, cloud->points[0].x);
    EXPECT_EQ(2, cloud->points[1].x);
    EXPECT_EQ(1, cloud->points[2].x);
}

TEST_F(evo_Tests, sortByEuclidDoesNotOverwriteInputParam)
{
    PCLPoint a, b, c;
    a.x = 1;
    b.x = 2;
    c.x = 3;

    PCLPoint target;
    target.x = 2.6;

    std::vector<PCLPoint> points;
    points.push_back(a);
    points.push_back(b);
    points.push_back(c);

    auto result = Util::sortByEuclidDistanceTo(points, target);

    EXPECT_EQ(1, points[0].x);

    EXPECT_EQ(3, result[0].x);
}

TEST_F(evo_Tests, ClipOfValueWorks)
{
    float input = 15.6;
    float threshold = 4.37;

    float result = Util::ClipOfValueAt(input, threshold);
    EXPECT_EQ(threshold, result);

}

TEST_F(evo_Tests, getAngleBetweenWorks)
{
    PCLPoint a, b;
    a.x = 1;
    a.y = 1;
    b.x = 5;
    b.y = 1;

    EXPECT_NEAR(-M_PI_2, Util::getAngleBetween(a, b), 0.000001);

    EXPECT_NEAR(M_PI_2, Util::getAngleBetween(b, a), 0.000001);

    b.y = 2;
    EXPECT_NEAR(-M_PI_2 + 0.24497866, Util::getAngleBetween(a, b), 0.0001);
}


TEST_F(evo_Tests, ClipOfValueKeepsSignCorrect)
{
    float input = -15.6;
    float threshold = 4.37;
    float result = Util::ClipOfValueAt(input, threshold);
    EXPECT_EQ(-threshold, result);
}

TEST_F(evo_Tests, ClipOfValueWorksWithNegativeNumbers)
{
    float input = -15.6;
    float threshold = 4.37;

    float result = Util::ClipOfValueAt(input, threshold);
    EXPECT_EQ(-threshold, result);
}

TEST_F(evo_Tests, ClipOfValueIgnoresThresholdSign)
{
    float input = 15.6;
    float threshold = -4.37;

    float result = Util::ClipOfValueAt(input, threshold);
    EXPECT_EQ(-threshold, result);
}

TEST_F(evo_Tests, ClipOfKeepsValuesBelowThreshold)
{
    float input = 4.37;
    float threshold = 15.6;

    float result = Util::ClipOfValueAt(input, threshold);
    EXPECT_EQ(input, result);
}