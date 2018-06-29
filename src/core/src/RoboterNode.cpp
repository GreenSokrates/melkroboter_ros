#include "core/RoboterNode.h"

RoboterNode::RoboterNode(ros::NodeHandle *nodehandle) : nh_(*nodehandle) {
  ROS_INFO("Constructed a RoboterNodeClass");
  initializeServices();
  initializePlanner();
}

void RoboterNode::initializeServices() {
  ROS_INFO("Initializing Services");
  MoveToPose_Service_ =
      nh_.advertiseService("/Melkroboter/MoveToPose",
                           &RoboterNode::RoboterNode::MoveToPose_cb_, this);
  MoveLinear_Service_ =
      nh_.advertiseService("/Melkroboter/MoveLinear",
                           &RoboterNode::RoboterNode::MoveLinear_cb_, this);
}

void RoboterNode::initializePlanner() {
  ROS_INFO("Setting up Planner");
  // Define MoveGroup and PlanningSceneInterface
  group.reset(
      new moveit::planning_interface::MoveGroupInterface("teatcup_urplaner"));
  // group->setPlannerId("RRTConnect");
  group->setEndEffectorLink("teatcup_tcp");
  group->setPlanningTime(1.5);
  group->setGoalJointTolerance(0.0001);
  group->setNumPlanningAttempts(3);
  group->setGoalOrientationTolerance(0.0001);
  group->setMaxVelocityScalingFactor(0.1);
}

bool RoboterNode::MoveToPose_cb_(core::moveToPose::Request &req,
                                 core::moveToPose::Response &res) {
  ROS_INFO("Moving to Pose");
  geometry_msgs::Pose myPose;
  myPose.position.x = req.x;
  myPose.position.y = req.y;
  myPose.position.z = req.z;
  myPose.orientation.w = req.oW;
  myPose.orientation.x = req.oX;
  myPose.orientation.y = req.oY;
  myPose.orientation.z = req.oZ;
  group->setPoseTarget(myPose);
  if (group->move()) {
    res.status = true;
  } else {
    res.status = false;
  }
  return 1;
}

bool RoboterNode::MoveLinear_cb_(core::moveLinear::Request &req,
                                 core::moveLinear::Response &res) {
  ROS_INFO("Moving Linear");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Melkroboter_RoboterNode");
  ros::NodeHandle nh;

  RoboterNode roboternode(&nh);

  ROS_INFO("RoboterNode is up");
  ros::spin();
  return 0;
}