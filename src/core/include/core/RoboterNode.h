#ifndef ROBOTER_NODE_H_
#define ROBOTER_NODE_H_

// ROS Specific includes
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/PointCloud2.h>
// Moveit Specific includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// include for Services
#include <core/TeatSearchService.h>
#include <core/getPose.h>
#include <core/moveLinear.h>
#include <core/moveToPose.h>
#include <core/startMelk.h>
// Includes for messages
#include <core/teatCount.h>

/**
 * @brief Class which contains all needed methods and variables to controll the robot for the milking system
 *
 */
class RoboterNode
{
public:
  RoboterNode(ros::NodeHandle *nodehandle);
  virtual ~RoboterNode(){};

  /**
   * @brief Moves the robot to a named pose
   *
   * @param poseName The name of the pose
   */
  void moveToNamed_(std::string poseName);

private:
  ros::NodeHandle nh_;

  // Service Servers
  ros::ServiceServer MoveToPose_Service_;
  ros::ServiceServer MoveLinear_Service_;
  ros::ServiceServer Start_Service_;
  ros::ServiceServer RobotPose_Service_;

  // Subscribers
  ros::Subscriber sub_teat_vl;
  ros::Subscriber sub_teat_vr;
  ros::Subscriber sub_teat_hl;
  ros::Subscriber sub_teat_hr;
  ros::Subscriber sub_teat_count;

  // Service Clients
  ros::ServiceClient TeatSearchClient_;

  // MoveGroup Interface
  boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;

  int teatCount = 0;
  int maxTeatCount = 0;
  std::vector<double> xVector;
  std::vector<double> yVector;
  std::vector<double> zVector;

  geometry_msgs::Pose teat_vl;
  geometry_msgs::Pose teat_vr;
  geometry_msgs::Pose teat_hl;
  geometry_msgs::Pose teat_hr;

  // Initializing Methods
  void initializeServices();
  void initializePlanner();
  void initializeServiceClients();
  void initializeSubscribers();

  /**
   * @brief Calls the Service to reset the TeatSearch
   *
   * @return true
   * @return false
   */
  bool callSearchStart_();

  /**
   * @brief Gets called when the startservice is called
   *
   * @param req ROS Param
   * @param res ROS Param
   * @return true If process was succesful
   * @return false If process was not succesfull
   */
  bool start_cb_(core::startMelk::Request &req, core::startMelk::Response &res);

  // Subscriber callbacks
  void teat_vl_cb_(geometry_msgs::PointStamped pointStamped);
  void teat_vr_cb_(geometry_msgs::PointStamped pointStamped);
  void teat_hl_cb_(geometry_msgs::PointStamped pointStamped);
  void teat_hr_cb_(geometry_msgs::PointStamped pointStamped);
  void teat_count_cb_(core::teatCount msg);

  // Service Callback Methods
  /**
   * @brief Wrapper for moveToPose_, gets called from the ROS-Service
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool moveToPose_cb_(core::moveToPose::Request &req, core::moveToPose::Response &res);

  /**
   * @brief Wrapper for the moveLinear_, gets called from the ROS-Service
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool moveLinear_cb_(core::moveLinear::Request &req, core::moveLinear::Response &res);

  /**
   * @brief Returns the current pose of the robot, gets called by ROS-Service
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool getPose_cb_(core::getPose::Request &req, core::getPose::Response &res);

  // Robot Movement Methods
  /**
   * @brief Moves the robot to the given pose
   *
   * @param position The position to move to
   * @return true Returns true if position is reached
   * @return false Returns false if the pose can't be reached
   */
  bool moveToPose_(geometry_msgs::Pose &position);

  /**
   * @brief Moves the robot linear to the given pose
   *
   * @param pose Pose to move to
   * @return true Returns true if the pose can be reached
   * @return false Returns false if the pose can't be reached
   */
  bool moveLinear_(geometry_msgs::Pose &pose);

  /**
   * @brief Moves the robot linear the given distance
   *
   * @param x distance in meters
   * @param y distance in meters
   * @param z distance in meters
   * @return true Returns true if the pose was reached
   * @return false Returns false if the pose can't be reached
   */
  bool moveLinear_(double x, double y, double z);

  /**
   * @brief Method to attatch teatcups
   *
   * Attatches the teatcups to the teats. Moves first to the pose + offset
   * then linear to the pose
   *
   * @param pose Pose of the teatcup
   * @param xOffset X-Offset in meters
   * @param yOffset Y-Offset in meters
   * @param zOffset Z-Offset in meters
   */
  void attatchTeatcup_(geometry_msgs::Pose &pose, float xOffset, float yOffset, float zOffset);
};

#endif /* ROBOTER_NODE_H_ */