#include <core/RemoveLegs.h>

RemoveLegs::RemoveLegs() {
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/melkroboter/cloud_no_legs", 1);
}

void RemoveLegs::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // used datasets
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<PointT>::Ptr cloud_reduced(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_reduced2(new pcl::PointCloud<PointT>());

  // Convert from ROS to PCL type
  pcl::fromROSMsg(*cloud_msg, *cloud);

  *cloud_reduced = removeLeg(cloud, 0);
  *cloud_reduced2 = removeLeg(cloud_reduced, 1);

  // Convert back to ROS data type
  pcl::toROSMsg(*cloud_reduced2, output);

  // Publish the data
  pub.publish(output);
}

pcl::PointCloud<PointT>
RemoveLegs::removeLeg(pcl::PointCloud<PointT>::Ptr &cloud, int leg) {
  // Used Datasets
  std::vector<int> minPoints;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_reduced(new pcl::PointCloud<PointT>);
  pcl::PointIndicesPtr cluster(new pcl::PointIndices);

  // Used Objects
  pcl::GridMinimum<PointT> minFilter(10);
  pcl::search::Search<PointT>::Ptr tree =
      boost::shared_ptr<pcl::search::Search<PointT> >(
          new pcl::search::KdTree<PointT>);
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
  pcl::RegionGrowing<PointT, pcl::Normal> reg;
  pcl::ExtractIndices<PointT> extract;

  // minFilter
  minFilter.setInputCloud(cloud);
  minFilter.filter(minPoints);

  // Growing
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(6.5 / 180.0 * M_PI);
  reg.setCurvatureThreshold(6.5);

  reg.getSegmentFromPoint(minPoints[leg], *cluster);

  extract.setInputCloud(cloud);
  extract.setIndices(cluster);
  extract.setNegative(true);
  extract.filter(*cloud_reduced);
  return *cloud_reduced;
}