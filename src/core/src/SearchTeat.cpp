#include <core/SearchTeat.h>

SearchTeat::SearchTeat(float gridSize_) {
  gridSize = gridSize_;
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/melkroboter/cloud_rotated", 1);
  // Create a ROS publisher for the output point cloud
  pub_point = nh.advertise<geometry_msgs::PointStamped>("/melkroboter/point_stamped", 1);
}

void SearchTeat::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // used datasets
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

  // Convert from ROS to PCL type
  pcl::fromROSMsg(*cloud_msg, *cloud);

  getMinPoints(cloud);
  for (size_t i = 0; i < teatCandidates.size(); i++) {
    isTeat(teatCandidates[i], cloud);
  }
}

void SearchTeat::getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud) {
  pcl::GridMinimum<PointT> gridMin(gridSize);
  gridMin.setInputCloud(cloud);
  gridMin.filter(teatCandidates);
}
bool SearchTeat::isTeat(int indexPoint, pcl::PointCloud<PointT>::Ptr &cloud) {
  // Objects
  pcl::KdTreeFLANN<PointT> kdtree;

  // Datasets
  bool isteat = false;
  geometry_msgs::PointStamped pointStamped;
  float teatRadius = 35.0 / 1000; // to get meter
  float teatHeight = 40.0 / 1000; // to get meter
  float teatHeightSq = teatHeight * teatHeight;
  float teatRadiusSq = teatRadius * teatRadius;
  float dot, dSq;
  Vec3 teatAxisVector;
  Vec3 searchPVec;
  PointT startP, searchP, endP;

  // Search for nearest Neighbour
  int K = 750; // number of neighours
  std::vector<int> pointIdxSearch(K);
  std::vector<float> pointIdxSquaredDistance(K);
  kdtree.setInputCloud(cloud);
  // Init teataxis
  teatAxisVector.x = 0.0f;
  teatAxisVector.y = 0.0f;
  teatAxisVector.z = teatHeight;
  // Init startPoint (lowes teat point)
  startP.x = cloud->points[indexPoint].x;
  startP.y = cloud->points[indexPoint].y;
  startP.z = cloud->points[indexPoint].z;
  // Init Endpoint
  endP.x = startP.x + teatAxisVector.x;
  endP.y = startP.y + teatAxisVector.y;
  endP.z = startP.z + teatAxisVector.z;

  // Search for neigbhour points
  if (kdtree.nearestKSearch(startP, K, pointIdxSearch,
                            pointIdxSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxSearch.size(); i++) {
      // Init searchpoint
      searchP.x = cloud->points[pointIdxSearch[i]].x;
      searchP.y = cloud->points[pointIdxSearch[i]].y;
      searchP.z = cloud->points[pointIdxSearch[i]].z;

      // Vector startpoint to searchpoint
      searchPVec.x = searchP.x - startP.x;
      searchPVec.y = searchP.y - startP.y;
      searchPVec.z = searchP.z - startP.z;

      // Dot Product between
      dot = (searchPVec.x * teatAxisVector.x) +
            (searchPVec.y * teatAxisVector.y) +
            (searchPVec.z * teatAxisVector.z);

      // if dot product is larger than zero and smaller than the squared
      // teatheight the point is between the lowest teat point and the end of
      // the defined cylinder
      if (dot < 0 || dot > teatHeightSq) {
        // Point is not insed of start & end bounds
        isteat = false;
        return isteat;
      } else {
        // Point is inside of Bounds
        dSq = (searchPVec.x * searchPVec.x + searchPVec.y * searchPVec.y +
               searchPVec.z * searchPVec.z) -
              (dot * dot) / teatHeightSq;
        if (dSq > teatRadiusSq) {
          isteat = false;
          return isteat;
        }
      }
    }
    isteat = true;
    pointStamped.point.x = startP.x;
    pointStamped.point.y = startP.y;
    pointStamped.point.z = startP.z;
    pub_point.publish(pointStamped);
    ROS_INFO("Teat found");
  }
}
