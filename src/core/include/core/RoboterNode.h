#ifndef ROBOTER_NODE_H_
#define ROBOTER_NODE_H_

// ROS Specific includes
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
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
  // core::TeatSearchService srv_teatSearch;
  // MoveGroup Interface
  boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;

  int teatCount = 0;
  int maxTeatCount = 0;
  std::vector<double> xVector;
  std::vector<double> yVector;
  std::vector<double> zVector;

  geometry_msgs::PointStamped teat_vl;
  geometry_msgs::PointStamped teat_vr;
  geometry_msgs::PointStamped teat_hl;
  geometry_msgs::PointStamped teat_hr;

  // Initialize Methods
  void initializeServices();
  void initializePlanner();
  void initializeServiceClients();
  void initializeSubscribers();

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

  void teat_vl_cb_(geometry_msgs::PointStamped pointStamped);
  void teat_vr_cb_(geometry_msgs::PointStamped pointStamped);
  void teat_hl_cb_(geometry_msgs::PointStamped pointStamped);
  void teat_hr_cb_(geometry_msgs::PointStamped pointStamped);
  void teat_count_cb_(core::teatCount msg);

  // Service Callback Methods
  bool moveToPose_cb_(core::moveToPose::Request &req, core::moveToPose::Response &res);
  bool moveLinear_cb_(core::moveLinear::Request &req, core::moveLinear::Response &res);
  bool getPose_cb_(core::getPose::Request &req, core::getPose::Response &res);

  // Robot Movement Methods
  bool moveToPose_(geometry_msgs::Pose &position);
  bool moveLinear_(geometry_msgs::Pose &pose);
  bool moveLinear_(double x, double y, double z);
  void attatchTeatcup_(geometry_msgs::Pose &pose, float xOffset, float yOffset, float zOffset);
};

#endif /* ROBOTER_NODE_H_ */