#ifndef SEARCH_TEAT_H_
#define SEARCH_TEAT_H_

// Comment out to disable seperate visualizer
#define enable_visualizer_

// ROS specific includes
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>

// PCL specific includes
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

// Includes for Service
#include <core/TeatSearchService.h>

typedef pcl::PointXYZ PointT;
struct Vec3
{
  float x;
  float y;
  float z;
};

class SearchTeat
{
public:
  SearchTeat(ros::NodeHandle *nodehandle, float gridSize, float teatDiameter, float teatLength);
  virtual ~SearchTeat(){};
  bool Searchloop();

private:
  // ROS nh, Publisher, Subscribers and Services
  ros::NodeHandle nh_;
  ros::Publisher pub_teat_1;
  ros::Publisher pub_teat_2;
  ros::Publisher pub_teat_3;
  ros::Publisher pub_teat_4;
  ros::Publisher pub_teat_counter;
  ros::Subscriber sub_;
  ros::ServiceServer searchTeat_service_;

  // Parameters for teatsearch
  float gridSize_;
  float teatDiameter_;
  float teatLength_;
  std::vector<int> teatCandidates;
  pcl::PointCloud<PointT>::Ptr cloud_;

  // Vectors to store the teat coordinates
  std::vector<double> xVector;
  std::vector<double> yVector;
  std::vector<double> zVector;
  int teatCount;

  // Initializing Methods
  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();

  void publishPoint(geometry_msgs::PointStamped pointStamped, int count);

  void getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud);

  void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  bool Service_cb_(core::TeatSearchService::Request &req, core::TeatSearchService::Response &res);
  bool isTeat(int indexPoint, pcl::PointCloud<PointT>::Ptr &cloud);

#ifdef enable_visualizer_
  boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer(std::string name);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  void updateCloud();
#endif
};

#endif /* SEARCH_TEAT_H_ */