#ifndef ROBOTER_NODE_H_
#define ROBOTER_NODE_H_

#include <ros/ros.h>
#include <ros/service.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// include for Services
#include <core/TeatSearchService.h>
#include <core/getPose.h>
#include <core/moveLinear.h>
#include <core/moveToPose.h>
#include <core/startMelk.h>

class RoboterNode
{
public:
  RoboterNode(ros::NodeHandle *nodehandle);
  virtual ~RoboterNode(){};
  void moveToNamed_(std::string poseName);

private:
  ros::NodeHandle nh_;

  // Service Servers
  ros::ServiceServer MoveToPose_Service_;
  ros::ServiceServer MoveLinear_Service_;
  ros::ServiceServer Start_Service_;
  ros::ServiceServer RobotPose_Service_;

  // Service Clients
  ros::ServiceClient TeatSearchClient_;
  // core::TeatSearchService srv_teatSearch;
  // MoveGroup Interface
  boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;

  int teatCount = 0;
  std::vector<double> xVector;
  std::vector<double> yVector;
  std::vector<double> zVector;

  // Initialize Methods
  void initializeServices();
  void initializePlanner();
  void initializeServiceClients();

  bool callTeatSearch_();

  bool start_cb_(core::startMelk::Request &req, core::startMelk::Response &res);

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