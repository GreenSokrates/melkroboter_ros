/**
 * @brief
 *
 * @file RotateCloud.cpp
 * @author Luis Meier
 * @date 2018-07-10
 */
#include <core/RotateCloud.h>

RotateCloud::RotateCloud(ros::NodeHandle *nodehandle, std::string in,
						 std::string out)
	: nh_(*nodehandle)
{
	listener = new tf::TransformListener();
	initializeSubscribers(in);
	initializePublishers(out);
}

void RotateCloud::initializeSubscribers(std::string in)
{
	sub_ = nh_.subscribe(in, 1, &RotateCloud::RotateCloud::cloud_cb_, this);
	ROS_INFO("Subscribing to: %s ", in.c_str());
}

void RotateCloud::initializePublishers(std::string out)
{
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out, 1);
	ROS_INFO("Publishing on: %s", out.c_str());
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
	sensor_msgs::PointCloud2 transformed;
	pcl::PCLPointCloud2::Ptr pclCloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// Wait if transform is not ready
	listener->waitForTransform("/world", (*cloud_msg).header.frame_id,
							   (*cloud_msg).header.stamp, ros::Duration(5.0));

	pcl_ros::transformPointCloud("/world", *cloud_msg, transformed, *listener);

	pcl_conversions::toPCL(transformed, *pclCloud);

	/*std::cerr << "PointCloud before filtering: "
			  << pclCloud->width * pclCloud->height << " data points ("
			  << pcl::getFieldsList(*pclCloud) << ")." << std::endl;
*/
	pcl::VoxelGrid<pcl::PCLPointCloud2> vxl;
	vxl.setInputCloud(pclCloud);
	vxl.setLeafSize(0.0025, 0.0025, 0.0025);
	vxl.filter(*cloud_filtered);
/*
	std::cerr << "PointCloud after filtering: "
			  << cloud_filtered->width * cloud_filtered->height
			  << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")."
			  << std::endl;
*/
	pcl_conversions::fromPCL(*cloud_filtered, output);

	pub_.publish(output);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RotateNode");
	ros::NodeHandle nh;

	std::string in;
	std::string out;
	std::string nodename = ros::this_node::getName();

	nh.getParam(nodename + "/input", in);
	nh.getParam(nodename + "/output", out);

	RotateCloud rotateCloud(&nh, in, out);

	ros::Rate loop_rate(30); // Freq in Hz
	ROS_INFO("RotateNode is up");
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
