/**
 * @brief
 *
 * @file SearchTeat.h
 * @author Luis Meier
 * @date 2018-07-09
 */

#ifndef SEARCH_TEAT_H_
#define SEARCH_TEAT_H_

// Comment out to disable seperate visualizer
#define enable_visualizer_

// ROS specific includes
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <Eigen/Core>

// Includes for Service
#include <core/TeatSearchService.h>

// Includes for messages
#include <core/teatCount.h>

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
  /**
   * @brief Searches for teats
   *
   * Everytime this function gets called it freezes the current Pointcloud and
   * searches in it for teats. The found teats
   * get published. This function should be called with the refresh rate of
   * the camera (Orbbec Astra 30Hz)
   */
  void Searchloop();

private:
  // ROS nh, Publisher, Subscribers and Services
  ros::NodeHandle nh_;
  ros::Publisher pub_teat_vl;
  ros::Publisher pub_teat_vr;
  ros::Publisher pub_teat_hl;
  ros::Publisher pub_teat_hr;
  ros::Publisher pub_teat_counter;
  ros::Subscriber sub_;
  ros::ServiceServer searchTeat_service_;

  // Parameters for teatsearch
  float gridSize_;
  float teatRadius_;
  float teatRadiusSq_;
  float teatLength_;

  /**
   * @brief Most recent cloud from the subscriber
   *
   */
  pcl::PointCloud<PointT>::Ptr cloud_;

  // Vectors to store the teat coordinates
  std::vector<double> xVector;
  std::vector<double> yVector;
  std::vector<double> zVector;
  int teatCount;
  int maxTeatCount;

  // Initializing ROS Methods

  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();

  bool segmentation(int seedIdxPoint, pcl::PointCloud<PointT>::Ptr &cloud);

  void validatePoint(int validatePointIdx, Vec3 &teatAxisVector, PointT &teatStartPoint,
					 pcl::PointCloud<PointT>::Ptr &cloud, bool &inheightbounds, bool &inradiusbounds);

  void updateTeatVector(Vec3 &teatAxisVector, std::vector<int> teatPoints, pcl::PointCloud<PointT>::Ptr &cloud,
						PointT &teatStartPoint);

  /**
   * @brief Publishes the Tip position of all 4 teats
   *
   * @param pointStamped
   * @param count
   */
  void publishTeats(pcl::PointCloud<PointT>::Ptr &cloud);

  /**
   * @brief Get the GridMin points from a given cloud
   *
   * @param cloud The input cloud (pcl::PointCloud<PointT>)
   * @return std::vector<int> The indexpoints for the minimal points
   */
  std::vector<int> getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud);

  /**
   * @brief Gets called everytime there is a new cloud published
   *
   * @param cloud_msg
   */
  void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  /**
   * @brief The callback for the advertised Service
   *
   * @param req The request msg
   * @param res The response msg
   * @return true If call was succsessful
   * @return false If call was not succsessful
   */
  bool Service_cb_(core::TeatSearchService::Request &req, core::TeatSearchService::Response &res);

  /**
   * @brief Checks if the given point is a Teat
   *
   * @param indexPoint The start point for the search
   * @param cloud	The cloud in witch should be searched
   * @return true If the start point matches to a teat
   * @return false If the start point does not match to a teat
   */
  bool isTeat(int indexPoint, pcl::PointCloud<PointT>::Ptr &cloud);

#ifdef enable_visualizer_
  boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer(std::string name);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  void updateCloud(std::vector<int> teatCandidates, pcl::PointCloud<PointT>::Ptr &cloud);

public:
  void spinViewer();
#endif
};

#endif /* SEARCH_TEAT_H_ */