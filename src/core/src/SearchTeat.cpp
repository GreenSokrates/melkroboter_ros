#include <core/SearchTeat.h>

SearchTeat::SearchTeat(ros::NodeHandle *nodehandle, double gridSize,
					   double teatDiameter, double teatLength,
					   double angleThreshold, double minDiameterThreshold,
					   double maxDiameterThreshold)
	: nh_(*nodehandle)
{
	// Parameters
	gridSize_ = gridSize;
	teatRadius_ = teatDiameter / 2;
	teatLength_ = teatLength;
	teatRadiusSq_ = teatRadius_ * teatRadius_;
	angleThreshold_ = angleThreshold * M_PI / 180.0f;
	minRadiusThreshold_ = minDiameterThreshold / 2;
	maxRadiusThreshold_ = maxDiameterThreshold / 2;

	// Inits for ROS
	initializePublishers();
	initializeSubscribers();
	initializeServices();

	cloud_.reset(new pcl::PointCloud<PointT>());
	cloud_freezed_.reset(new pcl::PointCloud<PointT>());

#ifdef enable_visualizer_
	viewer = createViewer("Cloud");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(0.1, "original", 0);
	viewer->setRepresentationToWireframeForAllActors();

#endif
}

void SearchTeat::initializeSubscribers()
{
	ROS_INFO("Initializing Subscribers");
	sub_ = nh_.subscribe("/cloud/RotatedFiltered", 1,
						 &SearchTeat::SearchTeat::cloud_cb_, this);
}

void SearchTeat::initializePublishers()
{
	ROS_INFO("Initializing Publishers");
	// Create a ROS publisher for the output point cloud
	pub_teat_vl =
		nh_.advertise<geometry_msgs::PointStamped>("/melkroboter/teats/vl", 1);
	pub_teat_vr =
		nh_.advertise<geometry_msgs::PointStamped>("/melkroboter/teats/vr", 1);
	pub_teat_hl =
		nh_.advertise<geometry_msgs::PointStamped>("/melkroboter/teats/hl", 1);
	pub_teat_hr =
		nh_.advertise<geometry_msgs::PointStamped>("/melkroboter/teats/hr", 1);
	pub_teat_counter =
		nh_.advertise<core::teatCount>("/melkroboter/teats/counter", 1);
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
	maxTeatCount = 0;
	return true;
}

void SearchTeat::cloud_cb_(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	// Convert from ROS to PCL type
	cloud_.reset(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(*cloud_msg, *cloud_);
}

void SearchTeat::Searchloop()
{
	double startTime = ros::Time::now().toSec();
	// used datasets
	std::vector<PointT> teatCandidates;

	// Counter for the teats and vectors to store the coordinates
	int teatCount_new = 0;
	std::vector<double> xVector_new;
	std::vector<double> yVector_new;
	std::vector<double> zVector_new;

	// Freeze the newest cloud to process it
	pcl::copyPointCloud(*cloud_, *cloud_freezed_);

#ifdef enable_visualizer_
	updateCloud(cloud_freezed_);
#endif /* enable_visualizer */

	// Search POI (minimas)
	teatCandidates = getMinPoints(cloud_freezed_);

	// goes through all possible teat candidates if one is a teat the
	// coordinates of the tip get pushed back into the vectors
	// and the teatCount gets incremented
	for (size_t i = 0; i < teatCandidates.size(); i++)
	{
		if (checkIfTeat(teatCandidates[i]))
		{
			teatCount_new++;
			xVector_new.push_back(teatCandidates[i].x);
			yVector_new.push_back(teatCandidates[i].y);
			zVector_new.push_back(teatCandidates[i].z);
			// addPoint(teatCandidates[i], "teat %lu" + i, 0, 0, 255, 0.008);
		}
	}
	// Replace the old teat vectors with the new ones
	xVector = xVector_new;
	yVector = yVector_new;
	zVector = zVector_new;
	// Replace the counter for the teats
	teatCount = teatCount_new;

	double duration = ros::Time::now().toSec() - startTime;
	if (teatCount > 4)
	{
		ROS_ERROR("Found more than 4 teats!");
		/*     std::cout << "xVector: " << xVector << "\n";
			std::cout << "yVector: " << yVector << "\n";
			std::cout << "zVector: " << zVector << "\n"; */
		viewer->spin();
	}
	// ROS_INFO("Found %i Teats in %f Seconds", teatCount, duration);

	// Publish the teats only if more than 3 found
	if (teatCount > 2)
		publishTeats(cloud_freezed_);
}

void SearchTeat::publishTeats(pcl::PointCloud<PointT>::Ptr &cloud)
{
	// Calculate the middle point between all teats
	PointT midPoint;
	for (size_t i = 0; i < teatCount; i++)
	{
		midPoint.x += xVector[i];
		midPoint.y += yVector[i];
	}
	midPoint.x = midPoint.x / teatCount;
	midPoint.y = midPoint.y / teatCount;

	geometry_msgs::PointStamped pointStamped;

	for (size_t i = 0; i < teatCount; i++)
	{
		pointStamped.header.frame_id = (*cloud).header.frame_id;
		pointStamped.header.stamp = pcl_conversions::fromPCL(
			(*cloud).header.stamp); // PCL header to ROS header
		pointStamped.point.x = xVector[i];
		pointStamped.point.y = yVector[i];
		pointStamped.point.z = zVector[i];
		if (xVector[i] < midPoint.x) // Front teats
		{
			if (yVector[i] < midPoint.y)
			{
				pub_teat_vl.publish(pointStamped);
			}
			else
			{
				pub_teat_vr.publish(pointStamped);
			}
		}
		else // Rear teats
		{
			if (yVector[i] < midPoint.y)
			{
				pub_teat_hl.publish(pointStamped);
			}
			else
			{
				pub_teat_hr.publish(pointStamped);
			}
		}
	}

	// Publish the teatcount and maxTeatCount
	if (teatCount > maxTeatCount)
		maxTeatCount = teatCount;
	core::teatCount msg;
	msg.count = teatCount;
	msg.maxCount = maxTeatCount;
	pub_teat_counter.publish(msg);
}

bool SearchTeat::checkIfTeat(PointT initialSeed)
{
	// controll
	int segments = 10;
	int k = 10; // neigbhours to search
	bool inHeightBounds = true;
	bool inRadiusBounds = true;
	int runcount = 0;

	// Setup KdTreeSearch
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud_freezed_);

	// Initializing seedpoint and mask
	PointT seedPoint;
	std::queue<PointT> searchMask;
	std::vector<int> teatSegmentIdx;
	searchMask.push(initialSeed);

	std::vector<std::vector<int>> heightSegmentsIdx(segments);

	std::vector<PointT> midpoints;
	std::vector<double> rMidpoints;

	// Reset teatAxisVector
	teatAxisVector_.x = 0.0f;
	teatAxisVector_.y = 0.0f;
	teatAxisVector_.z = teatLength_;

	// Aslong as there are Points in the searchMask we'll look for neighbours
	// and add them to the searchMask if they are inside the height and radius
	// bounds
	while (!searchMask.empty())
	{
		seedPoint = searchMask.front();
		searchMask.pop();

		// The vectors where KNN indexes are stored
		std::vector<int> neighbourIdx(k);
		std::vector<float> pointIdxSqDistance(k);

		// Search the K nearest neighbours
		if (kdtree.nearestKSearch(seedPoint, k, neighbourIdx,
								  pointIdxSqDistance) > 0)
		{
			runcount++;
			// Validate every found Neighbour
			for (size_t j = 0; j < neighbourIdx.size(); j++)
			{
				inHeightBounds = true;
				inRadiusBounds = true;
				validatePoint(neighbourIdx[j], initialSeed, inHeightBounds,
							  inRadiusBounds);

				// If a point is not inside the cylinder
				// it's definitely not a teat
				if (!inRadiusBounds)
				{
					continue;
				}
				else if (inHeightBounds)
				{
					// Check if point allready belongs to teat, if not add it to
					// SearchMask and TeatSegment
					if (std::find(teatSegmentIdx.begin(), teatSegmentIdx.end(),
								  neighbourIdx[j]) == teatSegmentIdx.end())
					{
						teatSegmentIdx.push_back(neighbourIdx[j]);
						searchMask.push(
							cloud_freezed_->points[neighbourIdx[j]]);
					}
				}
			}
		}
		if (runcount % 10 == 0 && runcount > 0)
		{
			segmentCloud(10, initialSeed, teatSegmentIdx, heightSegmentsIdx);
			midpoints.clear();
			radiusvec.clear();
			// midpoints.push_back(initialSeed_);
			for (size_t i = 0; i < heightSegmentsIdx.size(); i++)
			{
				MidpointNr = i;
				if (heightSegmentsIdx[i].size() > 20)
				{
					auto rvalue = calcMidpoint(initialSeed, segments,
											   heightSegmentsIdx[i]);
					midpoints.push_back(rvalue.first);
					radiusvec.push_back(rvalue.second);
				}
			}
			if (midpoints.size() > 1)
				updateTeatAxis(midpoints, initialSeed_);
		}
	}
	return simpleDecision(teatSegmentIdx.size(), radiusvec);
}

std::vector<PointT>
SearchTeat::getMinPoints(pcl::PointCloud<PointT>::Ptr &cloud)
{
	float distanceThreshold = 0.02;

	std::vector<int> teatCandidates;
	std::vector<PointT> search;
	std::vector<PointT> definitiveCandidates;
	PointT searchPoint;

	// Setup Gridminfilter
	pcl::GridMinimum<PointT> gridMin(gridSize_);
	gridMin.setInputCloud(cloud);
	gridMin.filter(teatCandidates);

	for (size_t i = 0; i < teatCandidates.size(); i++)
	{
		search.push_back(cloud->points[teatCandidates[i]]);
	}

	// for every teatcandidate
	for (size_t i = 0; i < search.size(); i++)
	{
		PointT pointSum = search[i];
		int sumcount = 1;
		// check if there is a near candidate and remove it
		for (size_t j = i + 1; j < search.size(); j++)
		{
			float xDist = fabs(search[i].x - search[j].x);
			float yDist = fabs(search[i].y - search[j].y);
			float zDist = fabs(search[i].z - search[j].z);
			float distance =
				sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));
			if (distance < distanceThreshold)
			{ // take the point with smaller z value
				if (pointSum.z < search[j].z)
				{
					pointSum.x = pointSum.x;
					pointSum.y = pointSum.y;
					pointSum.z = pointSum.z;
				}
				else
				{
					pointSum.x = search[j].x;
					pointSum.y = search[j].y;
					pointSum.z = search[j].z;
				}
				// Remove the merged Point
				search.erase(search.begin() + j);
			}
		}
		definitiveCandidates.push_back(pointSum);
	}
#ifdef enable_visualizer_
	addPointVec(teatCandidates, 0, 100, 100, 0.002);
	addPointVec(definitiveCandidates, 0, 255, 0, 0.004);
#endif

	return definitiveCandidates;
}

void SearchTeat::validatePoint(int validatePointIdx, PointT &teatStartPoint,
							   bool &inHeightBounds, bool &inRadiusBounds)
{
	PointT P_i;
	Vec3 v_i;
	double dotProduct, distanceSquared, teatLengthSq;

	// Init Vecotrs and Points
	// Init the current point to validate
	P_i.x = cloud_freezed_->points[validatePointIdx].x;
	P_i.y = cloud_freezed_->points[validatePointIdx].y;
	P_i.z = cloud_freezed_->points[validatePointIdx].z;

	// create a vector seedPoint(startPoint) to ValidatePoint
	v_i.x = P_i.x - teatStartPoint.x;
	v_i.y = P_i.y - teatStartPoint.y;
	v_i.z = P_i.z - teatStartPoint.z;

	// Calculate squared teathlength
	teatLengthSq = pow(teatAxisVector_.x, 2) + pow(teatAxisVector_.y, 2) +
				   pow(teatAxisVector_.z, 2);

	// Dot Product between TeatAxis and searchPointVector
	dotProduct = (v_i.x * teatAxisVector_.x) + (v_i.y * teatAxisVector_.y) +
				 (v_i.z * teatAxisVector_.z);

	distanceSquared = ((v_i.x * v_i.x) + (v_i.y * v_i.y) + (v_i.z * v_i.z)) -
					  (dotProduct * dotProduct) / teatLengthSq;

	// Check if the Point is inside length bounds
	if (dotProduct < 0 || dotProduct > teatLengthSq)
		inHeightBounds = false;

	// The point is outside of the radius bounds
	if (distanceSquared > teatRadiusSq_)
		inRadiusBounds = false;
}

std::pair<PointT, double> SearchTeat::calcMidpoint(PointT startPoint,
												   std::vector<int> &idxVector)
{

	PointT midpoint;
	double radius = 0;
	double z_sum = 0.0f;

	Eigen::MatrixXd A = Eigen::MatrixXd((idxVector.size()), 3);
	Eigen::VectorXd b = Eigen::VectorXd((idxVector.size()));
	Eigen::VectorXd c = Eigen::VectorXd(3);

	// Fill Matrix A and vector b
	for (size_t i = 0; i < idxVector.size(); i++)
	{
		z_sum += cloud_freezed_->points[idxVector[i]].z;

		// Fill the A-Matrix
		A(i, 0) = -2 * (cloud_freezed_->points[idxVector[i]].x);
		A(i, 1) = -2 * (cloud_freezed_->points[idxVector[i]].y);
		A(i, 2) = 1;

		// -(x.^2 + y.^2)
		b(i) = -(pow((cloud_freezed_->points[idxVector[i]].x), 2) +
				 pow((cloud_freezed_->points[idxVector[i]].y), 2));
	}
	// c(0) = x, c(1) = y, c(2) = x^2+y^2-c(2) = r^2
	// radius is sqrt((c(0)^2+c(1)^2)/4-c(2))
	c = A.colPivHouseholderQr().solve(b);

	midpoint.x = c(0);
	midpoint.y = c(1);
	midpoint.z = z_sum / idxVector.size();

	radius = sqrt(pow(c(0), 2) + pow(c(1), 2) - c(2));
	return std::make_pair(midpoint, radius);
	/* Avg Method
	double xavg = 0;
	double yavg = 0;
	double zavg = 0;
	for (size_t i = 0; i < idxVector.size(); i++){
		xavg += cloud_freezed_->points[idxVector[i]].x;
		yavg += cloud_freezed_->points[idxVector[i]].y;
		zavg += cloud_freezed_->points[idxVector[i]].z;
	}
	midpoint.x = xavg/idxVector.size();
	midpoint.y = yavg/idxVector.size();
	midpoint.z = zavg/idxVector.size();
	*/
}

void SearchTeat::updateTeatAxis(std::vector<PointT> &vec)
{
	size_t size = vec.size();
	Eigen::MatrixXd centers = Eigen::MatrixXd(vec.size(), 3);

	for (size_t i = 0; i < size; ++i)
	{
		centers(i, 0) = vec[i].x;
		centers(i, 1) = vec[i].y;
		centers(i, 2) = vec[i].z;
	}

	Eigen::Vector3d origin = centers.colwise().mean();
	Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
	Eigen::MatrixXd cov = centered.adjoint() * centered;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	Eigen::Vector3d axis = eig.eigenvectors().col(2).normalized();

	double xValue = axis(0);
	double yValue = axis(1);
	double zValue = axis(2);

	// To make sure the vector is allways pointing up and the right length of
	// teatLength_ as the values are normalized to 1meter
	if (zValue > 0)
	{
		teatAxisVector_.x = xValue * teatLength_;
		teatAxisVector_.y = yValue * teatLength_;
		teatAxisVector_.z = zValue * teatLength_;
	}
	else
	{
		teatAxisVector_.x = xValue * teatLength_ * (-1);
		teatAxisVector_.y = yValue * teatLength_ * (-1);
		teatAxisVector_.z = zValue * teatLength_ * (-1);
	}
}

void SearchTeat::segmentCloud(int segments, PointT startPoint,
							  std::vector<int> &teatSegmentIdx,
							  std::vector<std::vector<int>> &heightSegmentsIdx)
{
	double segmentLength = teatLength_ / segments;

	// push the points into the corresponding segments
	for (size_t i = 0; i < teatSegmentIdx.size(); i++)
	{
		for (size_t j = 0; j < segments; j++)
		{
			if (cloud_freezed_->points[teatSegmentIdx[i]].z >
					startPoint.z + segmentLength * j &&
				cloud_freezed_->points[teatSegmentIdx[i]].z <
					startPoint.z + segmentLength * (j + 1))
			{
				(heightSegmentsIdx[j]).push_back(teatSegmentIdx[i]);
			}
		}
	}
}

bool SearchTeat::simpleDecision(int pointcount, std::vector<double> &radiusvec)
{
	// Calculate avg Radius
	bool isteat = true;
	double sumRadius = 0;
	double avgRadius = 0;
	for (size_t i = 0; i < radiusvec.size(); i++)
	{
		sumRadius += radiusvec[i];
	}
	avgRadius = sumRadius / radiusvec.size();

	// Calculate Teatangle in x and y
	double teatAngle_x = 0;
	double teatAngle_y = 0;

	double length_x =
		sqrt(pow(teatAxisVector_.x, 2) + pow(teatAxisVector_.z, 2));
	double length_y =
		sqrt(pow(teatAxisVector_.y, 2) + pow(teatAxisVector_.z, 2));
	teatAngle_x = acos(teatAxisVector_.z / length_x);
	teatAngle_y = acos(teatAxisVector_.z / length_y);

	// Print all teatparameters
	/*
	std::cout << "TeatAngle in x is: " << teatAngle_x * 180 / M_PI << std::endl;
	std::cout << "TeatAngle in y is: " << teatAngle_y * 180 / M_PI << std::endl;
	std::cout << "AvgTeatradius is: " << avgRadius << std::endl;
	std::cout << "The TeatVector is : \nx: " << teatAxisVector_.x << std::endl
			  << "y: " << teatAxisVector_.y << std::endl
			  << "z: " << teatAxisVector_.z << std::endl;
	*/
	if (pointcount < 350)
	{
		std::cout << "Not enought points: " << pointcount << std::endl;
		isteat = false;
	}
	if (avgRadius < minRadiusThreshold)
	{
		std::cout << "Radius to small: " << avgRadius << std::endl;
		isteat = false;
	}
	if (avgRadius > maxRadiusThreshold)
	{
		std::cout << "Radius to large: " << avgRadius << std::endl;
		isteat = false;
	}
	if (teatAngle_x > angleThreshold)
	{
		std::cout << "X angle to wide: " << teatAngle_x * 180 / M_PI
				  << std::endl;
		isteat = false;
	}
	if (teatAngle_y > angleThreshold)
	{
		std::cout << "Y angle to wide: " << teatAngle_y * 180 / M_PI
				  << std::endl;
		isteat = false;
	}
	return isteat;
}

void SearchTeat::updateTeatAxis(std::vector<PointT> &vec, PointT &initialSeed_)
{
	size_t size = vec.size();
	Eigen::MatrixXd centers = Eigen::MatrixXd(vec.size(), 3);

	for (size_t i = 0; i < size; ++i)
	{
		centers(i, 0) = vec[i].x;
		centers(i, 1) = vec[i].y;
		centers(i, 2) = vec[i].z;
	}

	Eigen::Vector3d origin = centers.colwise().mean();
	Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
	Eigen::MatrixXd cov = centered.adjoint() * centered;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	Eigen::Vector3d axis = eig.eigenvectors().col(2).normalized();
	double xValue = axis(0);
	double yValue = axis(1);
	double zValue = axis(2);

	// To make sure the vector is allways pointing up
	if (zValue > 0)
	{
		teatAxisVector_.x = (xValue / 1000) * 40;
		teatAxisVector_.y = (yValue / 1000) * 40;
		teatAxisVector_.z = (zValue / 1000) * 40;
	}
	else
	{
		teatAxisVector_.x = (xValue / 1000) * -40;
		teatAxisVector_.y = (yValue / 1000) * -40;
		teatAxisVector_.z = (zValue / 1000) * -40;
	}
}

#ifdef enable_visualizer_
// Creates, initializes and returns a new viewer.
boost::shared_ptr<pcl::visualization::PCLVisualizer>
SearchTeat::createViewer(std::string name)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> v(
		new pcl::visualization::PCLVisualizer(name));
	return (v);
}

void SearchTeat::updateCloud(pcl::PointCloud<PointT>::Ptr &cloud)
{
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();
	viewer->addPointCloud(cloud, "cloud");
}

void SearchTeat::addPointVec(std::vector<int> teatCandidates, int r, int g,
							 int b, float size)
{
	for (size_t i = 0; i < teatCandidates.size(); i++)
	{
		std::stringstream stream;
		stream << "teatnrs " << i << rand();
		std::string candidateNr = stream.str();
		PointT point;
		point = cloud_freezed_->points[teatCandidates[i]];

		viewer->addSphere(point, size, r, g, b, candidateNr);
	}
	viewer->spinOnce();
}
void SearchTeat::addPointVec(std::vector<PointT> points, int r, int g, int b,
							 float size)
{
	for (size_t i = 0; i < points.size(); i++)
	{
		std::stringstream stream;
		stream << "teatnrs " << i << rand();
		std::string candidateNr = stream.str();

		viewer->addSphere(points[i], size, r, g, b, candidateNr);
	}
	viewer->spinOnce();
}

void SearchTeat::addPoint(PointT &point, std::string name, int r, int g, int b,
						  float size)
{
	viewer->addSphere(point, size, r, g, b, name);
	viewer->spinOnce();
}

void SearchTeat::spinViewer() { viewer->spinOnce(); }
#endif /* enable_visualizer_ */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SearchTeat_Node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
	spinner.start();

	SearchTeat searchTeat(&nh, 0.09, 0.04, 0.045);
	SearchTeat searchTeat(&nh, 0.06, 0.05, 0.04, 30, 0.02, 0.04);

	// GridSize, TeatDiameter, TeatLength

	ros::Rate loop_rate(30); // Freq in Hz
	ROS_INFO("SearchTeatNode is up");
	while (ros::ok())
	{
		ros::spinOnce();
		searchTeat.Searchloop();
#ifdef enable_visualizer_
		searchTeat.spinViewer();
#endif
		loop_rate.sleep();
	}

	return 0;
}