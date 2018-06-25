#ifndef DEPTH_FILTER_H_
#define DEPTH_FILTER_H_

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointXYZ PointT;

class DepthFilter {

public:
  DepthFilter();
  virtual ~DepthFilter(){};
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
};

#endif /*DEPTH_FILTER_H_ */