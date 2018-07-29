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
#include <cmath>
#include <iostream>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

#include <Eigen/Core>

// Includes for Service
#include <core/TeatSearchService.h>

// Includes for messages
#include <core/teatCount.h>

typedef pcl::PointXYZ PointT;
struct Vec3
{
	double x;
	double y;
	double z;
};

class SearchTeat
{
  public:
  /**
   * @brief Construct a new Search Teat object
   * 
   * @param nodehandle ROS Nodhandle
   * @param gridSize Gridsize for the gridminfilter
   * @param teatDiameter The Searchdiameter
   * @param teatLength The Searchlength
   * @param angleThreshold The maximum angle to the z Axis
   * @param minRadiusThreshold the minimum Radius a teat must have
   * @param maxRadiusThreshold the maximum Radius a teat must have
   */
	SearchTeat(ros::NodeHandle *nodehandle, double gridSize,
			   double teatDiameter, double teatLength, double angleThreshold,
			   double minDiameterThreshold, double maxDiameterThreshold);
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
	double gridSize_;
	double teatRadius_;
	double teatRadiusSq_;
	double teatLength_;
	double angleThreshold_;
	double minRadiusThreshold_;
	double maxRadiusThreshold_;

	pcl::PointCloud<PointT>::Ptr cloud_;
	pcl::PointCloud<PointT>::Ptr cloud_freezed_;

	// Vectors to store the teat coordinates
	std::vector<double> xVector;
	std::vector<double> yVector;
	std::vector<double> zVector;
	int teatCount;
	int maxTeatCount;
	Vec3 teatAxisVector_;

	// Initializing ROS
	void initializeSubscribers();
	void initializePublishers();
	void initializeServices();

	bool checkIfTeat(PointT initialSeed);

	/**
	 * @brief Checks if a point is in or outside a cylinder
	 *
	 * @param validatePointIdx The index of the point to validate
	 * @param teatStartPoint The start point of the cylinder
	 * @param inheightbounds Is set to true/false by the function
	 * @param inradiusbounds Is set to true/false by the funtion
	 */
	void validatePoint(int validatePointIdx, PointT &teatStartPoint,
					   bool &inheightbounds, bool &inradiusbounds);

	/**
	 * @brief Segments the given Points alon an Axis into n segments
	 *
	 * @param segments The number of segments to create
	 * @param startPoint the lowest Point in the Segment
	 * @param teatSegmentIdx The Index of the Points to segment
	 * @param heightSegmentsIdx The vector where the segments get stored to
	 */
	void segmentCloud(int segments, PointT startPoint,
					  std::vector<int> &teatSegmentIdx,
					  std::vector<std::vector<int>> &heightSegmentsIdx);

	/**
	 * @brief Calculates the Midpoint
	 *
	 * This Function approximates the Midpoint for a given circular segment of
	 * points
	 *
	 * @param startPoint
	 * @param idxVector
	 * @return std::pair<PointT, double> Midpoint position and radius
	 */
	std::pair<PointT, double> calcMidpoint(PointT startPoint,
										   std::vector<int> &idxVector);

	/**
	 * @brief updates the Teataxis
	 *
	 * Calculates the new orientation of the teataxis baesd on the given Points,
	 * which should be represented by the Calculated Mitdpoints. This method
	 * uses a least squares approximation
	 *
	 * @param vec PointT vector with the Midpoints
	 */
	void updateTeatAxis(std::vector<PointT> &vec);

	/**
	 * @brief Publishes the Tip position of all 4 teats
	 *
	 * @param pointStamped
	 * @param count
	 */
	void publishTeats(pcl::PointCloud<PointT>::Ptr &cloud);

	/**
	 * @brief Decides if the currently segmented Cloud is a teat or not
	 *
	 * @param pointcount The point count of the Segment
	 * @param radiusvec a double vector with the radius of the length segments
	 */
	bool simpleDecision(int pointcount, std::vector<double> &radiusvec);

	/**
	 * @brief Get the GridMin points from a given cloud
	 *
	 * @param cloud The input cloud (pcl::PointCloud<PointT>)
	 * @return std::vector<int> The indexpoints for the minimal points
	 */
	std::vector<PointT> getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud);

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
	bool Service_cb_(core::TeatSearchService::Request &req,
					 core::TeatSearchService::Response &res);

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
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
	createViewer(std::string name);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	void updateCloud(pcl::PointCloud<PointT>::Ptr &cloud);
	void addPoint(PointT &point, std::string name, int r, int g, int b,
				  float size);
	void addPointVec(std::vector<int> teatCandidates, int r, int g, int b,
					 float size);
	void addPointVec(std::vector<PointT> points, int r, int g, int b,
					 float size);

  public:
	void spinViewer();
#endif
};

#endif /* SEARCH_TEAT_H_ */