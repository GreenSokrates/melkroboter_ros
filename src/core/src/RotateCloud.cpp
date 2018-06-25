#include <core/RotateCloud.h>

RotateCloud::RotateCloud(float angle) {
    rotateAngle = angle;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/melkroboter/cloud_rotated", 1);
}

void RotateCloud::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);
  pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);

  // Transformation matrix object, initialized as identity
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  float theta = (rotateAngle) * (M_PI / 180.0f);

  // Rotation matrix (row, column)
  transformation(1, 1) = cos(theta);
  transformation(1, 2) = -sin(theta);
  transformation(2, 1) = sin(theta);
  transformation(2, 2) = cos(theta);

  // Rotate the cloud
  pcl::transformPointCloud(cloud, *cloud_transformed, transformation);

  // Convert back to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_transformed, output);

  // Publish the data
  pub.publish(output);
}