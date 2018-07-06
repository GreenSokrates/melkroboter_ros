#include <core/DepthFilter.h>
//#include <core/RemoveLegs.h>
#include <core/RotateCloud.h>
#include <core/SearchTeat.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "Melkroboter/SearchNode");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // CreateFilters
  DepthFilter depthFilter(&nh, -1.0, 1.0, -0.2, 0.8, 0.2, 0.8);
  RotateCloud rotateCloud(-65);
  //RemoveLegs removeLegs;
  

  // Create a ROS subscriber for the callbackfunctions in the classes

  ros::Subscriber sub_Rotation =
      nh.subscribe("/melkroboter/cloud_filtered", 1, &RotateCloud::cloud_cb,
                   &rotateCloud);
  /*ros::Subscriber sub_RemLegs = nh.subscribe(
      "/melkroboter/cloud_rotated", 1, &RemoveLegs::cloud_cb, &removeLegs);*/

  ROS_INFO("SearchNode ready");
  ros::Rate loop_rate(30); // 30Hz
  while (nh.ok()) {
    loop_rate.sleep();
  }

  return 0;
}