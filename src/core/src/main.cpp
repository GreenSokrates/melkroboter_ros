#include <core/DepthFilter.h>
#include <core/RemoveLegs.h>
#include <core/RotateCloud.h>
#include <core/SearchTeat.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "Melkroboter_core");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  DepthFilter depthFilter;
  RotateCloud rotateCloud_65 = (-65);
  RemoveLegs removeLegs;
  SearchTeat searchTeat = (0.08);

  // Create a ROS subscriber for the callbackfunctions in the classes
  ros::Subscriber sub_Filter = nh.subscribe(
      "/camera/depth/points", 1, &DepthFilter::cloud_cb, &depthFilter);
  ros::Subscriber sub_Rotation =
      nh.subscribe("/melkroboter/cloud_filtered_z", 1, &RotateCloud::cloud_cb,
                   &rotateCloud_65);
  ros::Subscriber sub_RemLegs = nh.subscribe(
      "/melkroboter/cloud_rotated", 1, &RemoveLegs::cloud_cb, &removeLegs);
  ros::Subscriber sub_TeatSearch = nh.subscribe(
      "/melkroboter/cloud_no_legs", 1, &SearchTeat::cloud_cb, &searchTeat);
      

  ros::Rate loop_rate(10);
      while (ros::ok()) {
        loop_rate.sleep();
      }
      return 1;
}