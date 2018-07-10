/**
 * @file DepthFilter.h
 * @author Luis Meier
 * @date 2018-07-09
 */

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

/**
 * @brief Class to Filter cloud
 *
 * This class uses three passthroughfilters (x,y,z) the min and max values for each filter have to be defined, aswell as
 * the subscribe and publish path
 */
class DepthFilter
{
public:
  /**
   * @brief Construct a new Depth Filter object
   *
   * @param nodehandle The ROS Nodehandle
   * @param subscribepath String with where to Subscribe
   * @param publishpath String with where to publish the filtered cloud
   * @param minX in meters
   * @param maxX in meters
   * @param minY in meters
   * @param maxY in meters
   * @param minZ in meters
   * @param maxZ in meters
   */
  DepthFilter(ros::NodeHandle *nodehandle, std::string subscribepath, std::string publishpath, float &minX, float &maxX,
              float &minY, float &maxY, float &minZ, float &maxZ);
  virtual ~DepthFilter(){};

private:
  // ROS specific
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  // Filter parameters
  std::string FieldDirection_;
  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;

  // Initializer Methods
  void initializePublishers(std::string &out);
  void initializeSubscribers(std::string &in);
  
  /**
   * @brief Callback function for filter
   *
   * This function gets called everytime a new cloud gets published to the subscribed topic
   *
   * @param cloud_msg
   */
  void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

#endif /*DEPTH_FILTER_H_ */