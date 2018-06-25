#include <core/DepthFilter.h>

DepthFilter::DepthFilter() {
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/melkroboter/cloud_filtered_z", 1);
}

void DepthFilter::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // perform filtering
  pcl::PassThrough<pcl::PCLPointCloud2> passFilter;
  passFilter.setInputCloud(cloudPtr);
  passFilter.setFilterFieldName("z");
  passFilter.setFilterLimits(0.2, 1.0);
  passFilter.filter(cloud_filtered); // Apply the filter

  // Convert back to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish(output);
}