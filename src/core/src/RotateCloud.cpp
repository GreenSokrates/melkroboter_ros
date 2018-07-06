#include <core/RotateCloud.h>

RotateCloud::RotateCloud(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{
	listener = new tf::TransformListener();
	initializePublishers();
	initializeSubscribers();
}

void RotateCloud::initializePublishers()
{
	pub_ =
		nh_.advertise<sensor_msgs::PointCloud2>("/cloud/filteredAndRotated", 1);
}

void RotateCloud::initializeSubscribers()
{
	sub_ = nh_.subscribe("/cloud/filtered", 1,
						 &RotateCloud::RotateCloud::cloud_cb_, this);
}
/*
void RotateCloud::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // used datasets
  pcl::PointCloud<PointT> cloud;
  pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);

  // Convert from ROS to PCL type
  pcl::fromROSMsg(*cloud_msg, cloud);

  // Transformation matrix object, initialized as identity
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  float theta = (rotateAngle) * (M_PI / 180.0f);

  // define rotation matrix (row, column)
  transformation(1, 1) = cos(theta);
  transformation(1, 2) = -sin(theta);
  transformation(2, 1) = sin(theta);
  transformation(2, 2) = cos(theta);

  // Rotate the cloud
  pcl::transformPointCloud(cloud, *cloud_transformed, transformation);

  // Convert back to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_transformed, output);

  // Publish the data
  pub.publish(output);
} */

void RotateCloud::cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	sensor_msgs::PointCloud2 output;

	listener->waitForTransform("/base", (*cloud_msg).header.frame_id,
							   (*cloud_msg).header.stamp, ros::Duration(5.0));

	pcl_ros::transformPointCloud("/base", *cloud_msg, output, *listener);

	pub_.publish(output);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RotateNode");
	ros::NodeHandle nh;
	RotateCloud rotateCloud(&nh);

	ros::Rate loop_rate(30); // Freq in Hz
	ROS_INFO("RotateNode is up");
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
