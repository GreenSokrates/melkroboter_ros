/**
 * @brief 
 * 
 * @file DepthFilter.h
 * @author your name
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
 * @brief 
 * 
 */
class DepthFilter
{

  public:
  /**
   * @brief Construct a new Depth Filter object
   * 
   * @param nodehandle The ROS Nodehandle
   * @param subscribepath String with where to Subscribe to
   * @param publishpath 
   * @param minX 
   * @param maxX 
   * @param minY 
   * @param maxY 
   * @param minZ 
   * @param maxZ 
   */
	DepthFilter(ros::NodeHandle *nodehandle, std::string subscribepath, std::string publishpath,
				float &minX, float &maxX, float &minY, float &maxY, float &minZ,
				float &maxZ);
	virtual ~DepthFilter(){};

  private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	std::string FieldDirection_;
	float minX_;
	float maxX_;
	float minY_;
	float maxY_;
	float minZ_;
	float maxZ_;
/**
 * @brief 
 * 
 * @param out 
 */
	void initializePublishers(std::string &out);
	void initializeSubscribers(std::string &in);
	void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

#endif /*DEPTH_FILTER_H_ */