#include "core/RoboterNode.h"

RoboterNode::RoboterNode(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{
  ROS_INFO("Constructed a RoboterNodeClass");
  initializeServices();
  initializePlanner();
  initializeServiceClients();
}

void RoboterNode::initializeServices()
{
  ROS_INFO("Initializing Services");
  MoveToPose_Service_ =
      nh_.advertiseService("/Melkroboter/MoveToPose", &RoboterNode::RoboterNode::moveToPose_cb_, this);
  MoveLinear_Service_ =
      nh_.advertiseService("/Melkroboter/MoveLinear", &RoboterNode::RoboterNode::moveLinear_cb_, this);
  Start_Service_ = nh_.advertiseService("/Melkroboter/Start", &RoboterNode::RoboterNode::start_cb_, this);
  RobotPose_Service_ = nh_.advertiseService("/Melkroboter/getPose", &RoboterNode::RoboterNode::getPose_cb_, this);
}

void RoboterNode::initializeServiceClients()
{
  TeatSearchClient_ = nh_.serviceClient<core::TeatSearchService>("/Melkroboter/SearchTeat");
  /*client_moveLinear_ =
      nh_.serviceClient<core::moveLinear>("/Melkroboter/MoveLinear");
  client_movePose_ =
      nh_.serviceClient<core::moveToPose>("/Melkroboter/MoveToPose");*/
}

void RoboterNode::initializePlanner()
{
  ROS_INFO("Setting up Planner");
  // Define MoveGroup and PlanningSceneInterface
  group.reset(new moveit::planning_interface::MoveGroupInterface("teatcup_urplaner"));
  // group->setPlannerId("RRTConnect");
  group->setEndEffectorLink("teatcup_tcp");
  group->setPlanningTime(3);
  group->setGoalJointTolerance(0.0001);
  group->setNumPlanningAttempts(3);
  group->setGoalOrientationTolerance(0.0001);
  group->setMaxVelocityScalingFactor(0.1);
}

bool RoboterNode::getPose_cb_(core::getPose::Request &req, core::getPose::Response &res)
{
  res.pose = (group->getCurrentPose(group->getEndEffectorLink())).pose;
  return true;
}

bool RoboterNode::moveToPose_cb_(core::moveToPose::Request &req, core::moveToPose::Response &res)
{
  ROS_INFO("Moving to Pose");
  geometry_msgs::Pose myPose;
  myPose.position.x = req.x;
  myPose.position.y = req.y;
  myPose.position.z = req.z;
  myPose.orientation.w = req.oW;
  myPose.orientation.x = req.oX;
  myPose.orientation.y = req.oY;
  myPose.orientation.z = req.oZ;

  res.status = moveToPose_(myPose);

  return true;
}

bool RoboterNode::moveLinear_cb_(core::moveLinear::Request &req, core::moveLinear::Response &res)
{
  if ((req.x || req.y || req.z) != 0)
  {
    moveLinear_(req.x, req.y, req.z);
  }
  else
  {
    moveLinear_(req.pose);
  }
  return true;
}

bool RoboterNode::start_cb_(core::startMelk::Request &req, core::startMelk::Response &res)
{
  ROS_INFO("Start called");
  moveToNamed_("search_ur");

  callTeatSearch_();

  geometry_msgs::Pose pose;
  pose.position.x = xVector[0];
  pose.position.y = yVector[0];
  pose.position.z = zVector[0] - 0.05;
  pose.orientation.w = 0.707;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = -0.707;

  moveToPose_(pose);
  ros::Duration(2).sleep();

  /*
    ROS_INFO("Going into for loop");
    for (size_t i = 0; i < srv_teatSearch.response.count; i++) {
      srv_moveToPose.request.x = xVector[i];
      srv_moveToPose.request.y = yVector[i];
      srv_moveToPose.request.z = zVector[i]-0.05;
      client_movePose_.call(srv_moveToPose);
      ros::Duration(2).sleep();
    } */
  res.outstatus = true;

  moveToNamed_("home_ur");
  return true;
}

bool RoboterNode::moveToPose_(geometry_msgs::Pose &pose)
{
  ROS_INFO("Moving to Pose");
  geometry_msgs::Pose tempPose = pose;

  group->setPoseTarget(tempPose);
  if (group->move())
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool RoboterNode::moveLinear_(geometry_msgs::Pose &pose)
{
  ROS_INFO("Moving Linear");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  std::vector<geometry_msgs::Pose> waypoints_tool;
  geometry_msgs::Pose start_pose = (group->getCurrentPose(group->getEndEffectorLink())).pose;
  waypoints_tool.push_back(pose);

  moveit_msgs::RobotTrajectory trajectory_msg;
  double fraction = group->computeCartesianPath(waypoints_tool,
                                                0.001, // eef_step
                                                5,     // jump_threshold
                                                trajectory_msg, true);
  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
  if (fraction >= 0.90)
  {
    group->execute(plan);
    return 1;
  }
  else
  {
    return 0;
  }
}

bool RoboterNode::moveLinear_(double x, double y, double z)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  std::vector<geometry_msgs::Pose> waypoints_tool;
  geometry_msgs::Pose test_pose = (group->getCurrentPose(group->getEndEffectorLink())).pose;

  test_pose.position.x += x;
  test_pose.position.y += y;
  test_pose.position.z += z;
  waypoints_tool.push_back(test_pose);

  moveit_msgs::RobotTrajectory trajectory_msg;
  double fraction = group->computeCartesianPath(waypoints_tool,
                                                0.001, // eef_step
                                                5,     // jump_threshold
                                                trajectory_msg, true);
  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
  if (fraction >= 0.90)
  {
    group->execute(plan);
    return 1;
  }
  else
  {
    return 0;
  }
}

void RoboterNode::moveToNamed_(std::string poseName)
{
  ROS_INFO("Called move to Named");
  group->setNamedTarget(poseName);
  group->move();
  ROS_INFO("Finished move Named");
}

void RoboterNode::attatchTeatcup_(geometry_msgs::Pose &pose, float xOffset, float yOffset, float zOffset)
{
  // move under teat
  geometry_msgs::Pose offsetPose;
  offsetPose.position.x = pose.position.x + xOffset;
  offsetPose.position.y = pose.position.y + yOffset;
  offsetPose.position.z = pose.position.z + zOffset;
  offsetPose.orientation = pose.orientation;

  moveToPose_(offsetPose);
  // linear up
  moveLinear_(pose);
  // linear down
  moveLinear_(offsetPose);
  return;
}

bool RoboterNode::callTeatSearch_()
{
  // Generate the service msg
  core::TeatSearchService srv_teatSearch;
  // Call the service
  TeatSearchClient_.call(srv_teatSearch);

  ROS_INFO("Found : %d", srv_teatSearch.response.count);
  while (srv_teatSearch.response.count != 4)
  {
    ROS_INFO("Second Time Found : %d", srv_teatSearch.response.count);
    ROS_INFO("Not 4 found, retrying");
    TeatSearchClient_.call(srv_teatSearch);
  }
  xVector = srv_teatSearch.response.x;
  yVector = srv_teatSearch.response.y;
  zVector = srv_teatSearch.response.z;
  ROS_INFO("Found : %d", srv_teatSearch.response.count);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Melkroboter_RoboterNode");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  RoboterNode roboternode(&nh);

  ros::Duration(5).sleep(); // sleep for half a second

  std::cout << "Roboter is ready to start!  \n\n"
            << "It will move to Home first\n"
            << "Ready and all clear? (y/n)" << std::endl;
  char response;
  std::cin >> response;
  if (response != 'y' && response != 'Y')
  {
    ROS_ERROR("Make sure all is clear!");
    ros::Duration(5).sleep();
    ros::shutdown();
    return -1;
  }

  roboternode.moveToNamed_("home_ur");

  ROS_INFO("RoboterNode is up");
  ros::Rate loop_rate(10); // Freq of 10 Hz
  while (ros::ok())
  {
    loop_rate.sleep();
  }

  return 0;
}