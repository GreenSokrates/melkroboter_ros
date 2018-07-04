#ifndef ROTATE_CLOUD_H_
#define ROTATE_CLOUD_H_

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

// PCL specific includes
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
  tf::TransformListener *listener;
};

#endif /* ROTATE_CLOUD_H_ */