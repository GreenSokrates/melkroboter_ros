#ifndef ROTATE_CLOUD_H_
#define ROTATE_CLOUD_H_

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;

class RotateCloud {
  float rotateAngle;
  Eigen::Matrix4f transformation;

public:
  RotateCloud(float);
  virtual ~RotateCloud(){};
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
};

#endif /* ROTATE_CLOUD_H_ */