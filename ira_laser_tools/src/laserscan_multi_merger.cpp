#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"

using namespace std;
using namespace pcl;
//using namespace laserscan_multi_merger;

namespace ira_laser_tools{
class LaserscanMerger
{
	public:
		LaserscanMerger();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic);

	private:
		ros::NodeHandle node_;
		laser_geometry::LaserProjection projector_;
		tf::TransformListener tfListener_;

		ros::Publisher point_cloud_publisher_;
		vector<ros::Subscriber> scan_subscribers;
		vector<bool> clouds_modified;

		vector<pcl::PCLPointCloud2> clouds;
		vector<string> input_topics;

		void laserscan_topic_parser();

		double angle_min;
		double angle_max;
		double angle_increment;
		double time_increment;
		double scan_time;
		double range_min;
		double range_max;

		string destination_frame;
		string cloud_destination_topic;
		string laserscan_topics;

		std::string const tag_ = "[LASER-SCAN-MULTI-MERGER]";
};


void LaserscanMerger::laserscan_topic_parser()
{
	ros::master::V_TopicInfo topics;

	istringstream iss(laserscan_topics);
	set<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), inserter<set<string>>(tokens, tokens.begin()));
	vector<string> tmp_input_topics;

	while (!tokens.empty())
	{
		ROS_INFO("%s Waiting for topics ...", tag_.c_str());
		ros::master::getTopics(topics);
		sleep(1);

		for (int i = 0; i < topics.size(); i++)
		{
			if (topics[i].datatype == "sensor_msgs/LaserScan" && tokens.erase(topics[i].name) > 0)
			{
				tmp_input_topics.push_back(topics[i].name);
			}
		}
	}

	sort(tmp_input_topics.begin(), tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

	// Do not re-subscribe if the topics are the same
	if ((tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for (int i = 0; i < scan_subscribers.size(); i++)
			scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;

		if (input_topics.size() > 0)
		{
			scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
			ROS_INFO("%s Subscribing to topics\t%ld", tag_.c_str(), scan_subscribers.size());
			for (int i = 0; i < input_topics.size(); ++i)
			{
				scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan>(input_topics[i].c_str(), 1, boost::bind(&LaserscanMerger::scanCallback, this, _1, input_topics[i]));
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
			ROS_INFO("%s Not subscribed to any topic.", tag_.c_str());
	}
}

LaserscanMerger::LaserscanMerger()
{
	ros::NodeHandle nh("~");

	nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
	nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
	nh.param<std::string>("laserscan_topics", laserscan_topics, "");
	nh.param("angle_min", angle_min, -M_PI);
	nh.param("angle_max", angle_max, M_PI);
	nh.param("angle_increment", angle_increment, 0.0058);
	nh.param("scan_time", scan_time, 0.0333333);
	nh.param("range_min", range_min, 0.01);
	nh.param("range_max", range_max, 50.0);

	this->laserscan_topic_parser();

	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(cloud_destination_topic.c_str(), 1, false);
}

void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic)
{
	sensor_msgs::PointCloud tmpCloud1, tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

	// refer to http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29
	try
	{
		// Verify that TF knows how to transform from the received scan to the destination scan frame
		tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));
		projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}
	catch (tf::TransformException ex)
	{
		return;
	}

	for (int i = 0; i < input_topics.size(); i++)
	{
		if (topic.compare(input_topics[i]) == 0)
		{
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2, tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}

	// Count how many scans we have
	int totalClouds = 0;
	for (int i = 0; i < clouds_modified.size(); i++)
		if (clouds_modified[i])
			totalClouds++;

	// Go ahead only if all subscribed scans have arrived
	if (totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for (int i = 1; i < clouds_modified.size(); i++)
		{
			#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
				pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
			#else
				pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			#endif
				clouds_modified[i] = false;
		}

		point_cloud_publisher_.publish(merged_cloud);
	}
}

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_multi_merger");

	ira_laser_tools::LaserscanMerger _laser_merger;

	ros::spin();

	return 0;
}
