#ifndef REMOVE_LEGS_H_
#define REMOVE_LEGS_H_

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

class RemoveLegs {
public:
  RemoveLegs();
  virtual ~RemoveLegs(){};
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  pcl::PointCloud<PointT> removeLeg(pcl::PointCloud<PointT>::Ptr &cloud,
                                    int leg);
};

#endif /* REMOVE_LEGS_H_ */