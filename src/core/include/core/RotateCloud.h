#ifndef ROTATE_CLOUD_H_
#define ROTATE_CLOUD_H_

// ROS specific includes
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
// Voxel grid
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
/**
 * @brief Rotates the input cloud into the world coordinate system and publishes it in the output msg
 *
 */
class RotateCloud
{
  float rotateAngle;
  Eigen::Matrix4f transformation;

public:
  RotateCloud(ros::NodeHandle *nodehandle, std::string in, std::string out);
  virtual ~RotateCloud(){};

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  tf::TransformListener *listener;

  // Initializing Methods
  void initializeSubscribers(std::string in);
  void initializePublishers(std::string out);

  void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

#endif /* ROTATE_CLOUD_H_ */