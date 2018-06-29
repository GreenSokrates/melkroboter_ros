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
  SearchTeat(ros::NodeHandle *nodehandle, float gridSize, float teatDiameter,
             float teatLength);
  virtual ~SearchTeat(){};

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_point_;
  ros::Subscriber sub_;
  float gridSize_;
  float teatDiameter_;
  float teatLength_;
  std::vector<int> teatCandidates;

  void initializeSubscribers();
  void initializePublishers();
  void getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud);
  void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  bool isTeat(int indexPoint, pcl::PointCloud<PointT>::Ptr &cloud);
};

#endif /* SEARCH_TEAT_H_ */