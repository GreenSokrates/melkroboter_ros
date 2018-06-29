#include <core/SearchTeat.h>

SearchTeat::SearchTeat(ros::NodeHandle *nodehandle, float gridSize,
                       float teatDiameter, float teatLength)
    : nh_(*nodehandle) {
  gridSize_ = gridSize;
  teatDiameter_ = teatDiameter;
  teatLength_ = teatLength;
  initializePublishers();
  initializeSubscribers();
}

void SearchTeat::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers");
  sub_ = nh_.subscribe("/melkroboter/cloud_no_legs", 1,
                       &SearchTeat::SearchTeat::cloud_cb_, this);
}

void SearchTeat::initializePublishers() {
  ROS_INFO("Initializing Publishers");
  // Create a ROS publisher for the output point cloud
  pub_point_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/melkroboter/point_stamped", 1, true);
}

void SearchTeat::cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
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
  pcl::GridMinimum<PointT> gridMin(gridSize_);
  gridMin.setInputCloud(cloud);
  gridMin.filter(teatCandidates);
}

bool SearchTeat::isTeat(int indexPoint, pcl::PointCloud<PointT>::Ptr &cloud) {
  // Objects
  pcl::KdTreeFLANN<PointT> kdtree;

  // Datasets
  bool isteat = false;
  geometry_msgs::PointStamped pointStamped;
  float teatRadius = teatDiameter_ / 2000; // to get meter and radius
  float teatLength = teatLength_ / 1000;   // to get meter
  float teatLengthSq = teatLength * teatLength;
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
  teatAxisVector.z = teatLength;
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
      // teatLength the point is between the lowest teat point and the end of
      // the defined cylinder
      if (dot < 0 || dot > teatLengthSq) {
        // Point is not insed of start & end bounds
        isteat = false;
        return isteat;
      } else {
        // Point is inside of Bounds
        dSq = (searchPVec.x * searchPVec.x + searchPVec.y * searchPVec.y +
               searchPVec.z * searchPVec.z) -
              (dot * dot) / teatLengthSq;
        if (dSq > teatRadiusSq) {
          isteat = false;
          return isteat;
        }
      }
    }
    isteat = true;
    pointStamped.header.frame_id = "/camera_depth_frame";
    pointStamped.header.stamp = ros::Time::now();
    pointStamped.point.x = startP.x;
    pointStamped.point.y = startP.y;
    pointStamped.point.z = startP.z;
    pub_point_.publish(pointStamped);
    ROS_INFO("Teat found");
  }
}
