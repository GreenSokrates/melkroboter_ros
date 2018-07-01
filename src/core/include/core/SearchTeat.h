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

// Includes for Service
#include <core/TeatSearchService.h>

#include <pcl/visualization/pcl_visualizer.h>

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
  ros::ServiceServer searchTeat_service_;

  float gridSize_;
  float teatDiameter_;
  float teatLength_;
  std::vector<int> teatCandidates;
  pcl::PointCloud<PointT>::Ptr cloud;


  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();

  pcl::PointCloud<PointT>::Ptr cloud;
  void getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud);
  void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  bool Service_cb_(core::TeatSearchService::Request &req,
                               core::TeatSearchService::Response &res);
  bool isTeat(int indexPoint, pcl::PointCloud<PointT>::Ptr &cloud);
};

#endif /* SEARCH_TEAT_H_ */