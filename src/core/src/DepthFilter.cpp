/**
 * @brief 
 * 
 * @file DepthFilter.cpp
 * @author Luis Meier
 * @date 2018-07-09
 */
#include <core/DepthFilter.h>

DepthFilter::DepthFilter(ros::NodeHandle *nodehandle, std::string subscribepath,
						 std::string publishpath, float &minX, float &maxX, float &minY,
						 float &maxY, float &minZ, float &maxZ)
	: nh_(*nodehandle)
{

	minX_ = minX;
	maxX_ = maxX;
	minY_ = minY;
	maxY_ = maxY;
	minZ_ = minZ;
	maxZ_ = maxZ;
	initializeSubscribers(subscribepath);
	initializePublishers(publishpath);
	ROS_INFO("Setting up filter with: X-> %f, %f, Y -> %f, %f, Z-> %f, %f",
			 minX_, maxX_, minY_, maxY_, minZ_, maxZ_);
}

void DepthFilter::initializeSubscribers(std::string &in)
{
	sub_ = nh_.subscribe(in, 1, &DepthFilter::DepthFilter::cloud_cb_, this);
	ROS_INFO("Subscribing on: %s", in.c_str());
}

void DepthFilter::initializePublishers(std::string &out)
{
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out, 1, true);
	ROS_INFO("Publishing on: %s", out.c_str());
}

void DepthFilter::cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	// Container for original & filtered data
	pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_x(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_y(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_z(new pcl::PointCloud<PointT>());

	// Convert to PCL data type
	pcl::fromROSMsg(*cloud_msg, *cloud_in);

	pcl::PassThrough<PointT> passFilter;

	// filter in x
	passFilter.setInputCloud(cloud_in);
	passFilter.setFilterFieldName("x");
	passFilter.setFilterLimits(minX_, maxX_);
	passFilter.filter(*cloud_x); // Apply the filter

	// filter in y
	passFilter.setInputCloud(cloud_x);
	passFilter.setFilterFieldName("y");
	passFilter.setFilterLimits(minY_, maxY_);
	passFilter.filter(*cloud_y); // Apply the filter

	// filter in z
	passFilter.setInputCloud(cloud_y);
	passFilter.setFilterFieldName("z");
	passFilter.setFilterLimits(minZ_, maxZ_);
	passFilter.filter(*cloud_z); // Apply the filter

	// Convert back to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_z, output);

	// Publish the data
	pub_.publish(output);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DepthFilterNode");
	ros::NodeHandle nh;

	float minX, minY, minZ, maxX, maxY, maxZ;
	std::string in;
	std::string out;
	std::string nodename = ros::this_node::getName();
	bool status;

	// Getting the Parameters from the launch file
	nh.getParam(nodename + "/minX", minX);
	nh.getParam(nodename + "/maxX", maxX);
	nh.getParam(nodename + "/minY", minY);
	nh.getParam(nodename + "/maxY", maxY);
	nh.getParam(nodename + "/minZ", minZ);
	nh.getParam(nodename + "/maxZ", maxZ);
	nh.getParam(nodename + "/input", in);
	nh.getParam(nodename + "/output", out);

	DepthFilter depthFilter(&nh, in, out, minX, maxX, minY, maxY, minZ, maxZ);

	ros::Rate loop_rate(30); // Freq in Hz
	ROS_INFO("DepthFilterNode is up");
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}