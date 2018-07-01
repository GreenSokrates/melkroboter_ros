#include <core/DepthFilter.h>

DepthFilter::DepthFilter(ros::NodeHandle *nodehandle, float minX, float maxX,
                         float minY, float maxY, float minZ, float maxZ)
    : nh_(*nodehandle)
{
  minX_ = minX;
  maxX_ = maxX;
  minY_ = minY;
  maxY_ = maxY;
  minZ_ = minZ;
  maxZ_ = maxZ;
  initializePublishers();
  initializeSubscribers();
}

void DepthFilter::initializePublishers()
{
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/melkroboter/cloud_filtered",
                                                 1, true);
}

void DepthFilter::initializeSubscribers()
{
  sub_ = nh_.subscribe("/camera/depth_registered/points", 1,
                       &DepthFilter::DepthFilter::cloud_cb_, this);
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