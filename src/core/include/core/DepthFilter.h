#ifndef DEPTH_FILTER_H_
#define DEPTH_FILTER_H_

// ROS specific includess
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
  DepthFilter(ros::NodeHandle *nodehandle, float minX, float maxX, float minY,
              float maxY, float minZ, float maxZ);
  virtual ~DepthFilter(){};

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::string FieldDirection_;
  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;

  void initializePublishers();
  void initializeSubscribers();
  void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

#endif /*DEPTH_FILTER_H_ */