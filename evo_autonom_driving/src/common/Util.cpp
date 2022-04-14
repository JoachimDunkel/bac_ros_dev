#include <algorithm>
#include <pcl/common/distances.h>
#include <evo_autonom_driving/common/Util.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace evo_autonom_driving {

    std::vector<PCLPoint> Util::sortByEuclidDistanceTo(std::vector<PCLPoint> points, const PCLPoint& target){
        std::sort(begin(points), end(points),
            [target](const PCLPoint& lhs, const PCLPoint& rhs)
            { return pcl::euclideanDistance(target, lhs) < pcl::euclideanDistance(target, rhs); });
        return points;
    }

    float Util::ClipOfValueAt(float value, float threshold) {
        threshold = fabs(threshold);
        float absolute_value = fabs(value);
        if(absolute_value > threshold){
            if(value < 0){
                value = -threshold;
            }
            else{
                value = threshold;
            }
        }
        return value;
    }

    void Util::sortByEuclidDistanceTo(PCLCloud::Ptr cloud, const PCLPoint& target){
        int num_points = cloud->points.size();
        std::vector<PCLPoint> points;
        for (int i = 0; i < num_points; i++)
        {
            points.push_back(cloud->points[i]);
        }

        points = sortByEuclidDistanceTo(points, target);

        cloud->points.clear();
        
        for (int i = 0; i < num_points; i++)
        {
            cloud->points.push_back(points[i]);
        }        
    }

    geometry_msgs::Quaternion Util::quaternionFromRPY(float roll, float pitch, float yaw) {
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(roll, pitch, yaw);

        return tf2::toMsg(quat_tf);
    }

    float Util::getAngleBetween(PCLPoint a, PCLPoint b) {
        //since we are working in robot coordinate systen which is 90° rotated counter clockwise.
        //x = -y
        //y = x
        return atan2(a.x - b.x, -a.y  + b.y);
    }

    geometry_msgs::Pose Util::extractPoseFrom(Eigen::Matrix4f transformation)
    {
        geometry_msgs::Pose pose;
        
        pose.position.x = transformation(0,3);
        pose.position.y = transformation(1,3);
        pose.position.z = 0;

        float yaw = atan2(transformation(1,0), transformation(0,0));

        tf2::Quaternion quaternion;
        quaternion.setRPY(0,0,yaw);

        pose.orientation = tf2::toMsg(quaternion);
        return pose;
    }

};