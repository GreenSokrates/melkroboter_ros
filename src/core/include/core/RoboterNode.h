#ifndef ROBOTER_NODE_H_
#define ROBOTER_NODE_H_

#include <ros/ros.h>
#include <ros/service.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// include for Services
#include <core/moveLinear.h>
#include <core/moveToPose.h>

class RoboterNode
{
public:
  RoboterNode(ros::NodeHandle *nodehandle);
  virtual ~RoboterNode(){};
  bool moveToNamed(std::string poseName);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer MoveToPose_Service_;
  ros::ServiceServer MoveLinear_Service_;
  boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;

  void initializeServices();
  void initializePlanner();
  bool moveToPose_cb_(core::moveToPose::Request &req, core::moveToPose::Response &res);
  bool moveLinear_cb_(core::moveLinear::Request &req, core::moveLinear::Response &res);

};

#endif /* ROBOTER_NODE_H_ */