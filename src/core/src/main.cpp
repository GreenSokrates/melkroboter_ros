#include <core/DepthFilter.h>
#include <core/RotateCloud.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "Melkroboter_core");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  DepthFilter depthFilter;
  RotateCloud rotateCloud = (-65);
  // Create a ROS subscriber for the callbackfunctions in the classes
  ros::Subscriber sub_Filter = nh.subscribe(
      "/camera/depth/points", 1, &DepthFilter::cloud_cb, &depthFilter);
  ros::Subscriber sub_Rotation = nh.subscribe(
      "/melkroboter/cloud_filtered_z", 1, &RotateCloud::cloud_cb, &rotateCloud);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    loop_rate.sleep();
  }
  return 1;
}