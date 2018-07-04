#include <core/SearchTeat.h>

SearchTeat::SearchTeat(ros::NodeHandle *nodehandle, float gridSize,
                       float teatDiameter, float teatLength)
    : nh_(*nodehandle)
{
  gridSize_ = gridSize;
  teatDiameter_ = teatDiameter;
  teatLength_ = teatLength;

  initializePublishers();
  initializeSubscribers();
  initializeServices();
}

void SearchTeat::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  sub_ = nh_.subscribe("/melkroboter/cloud_no_legs", 1,
                       &SearchTeat::SearchTeat::cloud_cb_, this);
}

void SearchTeat::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  // Create a ROS publisher for the output point cloud
  pub_point_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/melkroboter/point_stamped", 1, true);
}

void SearchTeat::initializeServices()
{
  ROS_INFO("Initializing Services");
  searchTeat_service_ = nh_.advertiseService(
      "/Melkroboter/SearchTeat", &SearchTeat::SearchTeat::Service_cb_, this);
}

bool SearchTeat::Service_cb_(core::TeatSearchService::Request &req,
                             core::TeatSearchService::Response &res)
{
  ROS_INFO("Service CB");
  // Cloud datasets
  pcl::PointCloud<PointT>::Ptr cloud_original(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_freezed(new pcl::PointCloud<PointT>());

  // Vector and Counter used for service response
  int teatCount = 0;
  std::vector<double> xVector;
  std::vector<double> yVector;
  std::vector<double> zVector;

  pcl::copyPointCloud(*cloud_, *cloud_freezed);
  getMinPoints(cloud_freezed);

  // Rotate Cloud
  //*cloud_original = SearchTeat::rotateCloudBack(*cloud_freezed);
  int j = 0;
  for (size_t i = 0; i < teatCandidates.size(); i++)
  {
    if (isTeat(teatCandidates[i], cloud_freezed))
    {
      teatCount++;
      xVector.push_back(-(cloud_freezed->points[teatCandidates[i]].x));
      yVector.push_back(-(cloud_freezed->points[teatCandidates[i]].y));
      zVector.push_back(cloud_freezed->points[teatCandidates[i]].z);
    }
  }
  res.count = teatCount;
  res.x = xVector;
  res.y = yVector;
  res.z = zVector;
  return true;
}

void SearchTeat::cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // Convert from ROS to PCL type
  cloud_.reset(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*cloud_msg, *cloud_);
}

void SearchTeat::getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud)
{
  pcl::GridMinimum<PointT> gridMin(gridSize_);
  gridMin.setInputCloud(cloud);
  gridMin.filter(teatCandidates);
}

bool SearchTeat::isTeat(int indexPoint, pcl::PointCloud<PointT>::Ptr &cloud)
{
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
                            pointIdxSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxSearch.size(); i++)
    {
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
      if (dot < 0 || dot > teatLengthSq)
      {
        // Point is not insed of start & end bounds
        isteat = false;
        return isteat;
      }
      else
      {
        // Point is inside of Bounds
        dSq = (searchPVec.x * searchPVec.x + searchPVec.y * searchPVec.y +
               searchPVec.z * searchPVec.z) -
              (dot * dot) / teatLengthSq;
        if (dSq > teatRadiusSq)
        {
          isteat = false;
          return isteat;
        }
      }
    }
    isteat = true;
    pointStamped.header.frame_id = (*cloud).header.frame_id;
    pointStamped.header.stamp = ros::Time::now();
    pointStamped.point.x = startP.x;
    pointStamped.point.y = startP.y;
    pointStamped.point.z = startP.z;
    pub_point_.publish(pointStamped);
    ROS_INFO("Teat Found");
    return isteat;
  }
}

pcl::PointCloud<PointT>
SearchTeat::rotateCloudBack(pcl::PointCloud<PointT> cloud)
{

  pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);

  // Transformation matrix object, initialized as identity
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  float theta = (-65) * (M_PI / 180.0f);

  // define rotation matrix (row, column)
  transformation(1, 1) = cos(theta);
  transformation(1, 2) = -sin(theta);
  transformation(2, 1) = sin(theta);
  transformation(2, 2) = cos(theta);

  // Rotate the cloud
  pcl::transformPointCloud(cloud, *cloud_transformed, transformation);

  return *cloud_transformed;
}
