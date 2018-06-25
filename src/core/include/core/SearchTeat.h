#ifndef SEARCH_TEAT_H_
#define SEARCH_TEAT_H_

// ROS specific includes
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/filters/grid_minimum.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZ PointT;
struct Vec3 {
  float x;
  float y;
  float z;
};

class SearchTeat {
public:
  SearchTeat(float);
  virtual ~SearchTeat(){};
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Publisher pub_point;
  ros::Subscriber sub;
  float gridSize;
  float teatDiameter;
  float teatLength;
  std::vector<int> teatCandidates;

  void getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud);
  bool isTeat(int indexPoint, pcl::PointCloud<PointT>::Ptr &cloud);
};

#endif /* SEARCH_TEAT_H_ */