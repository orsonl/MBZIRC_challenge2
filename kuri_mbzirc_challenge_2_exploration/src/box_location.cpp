#include <kuri_mbzirc_challenge_2_exploration/box_location.h>

BoxLocator::BoxLocator(actionlib::SimpleActionServer<ServerAction>* actionserver, bool bypass)
{
  as_ = actionserver;
  bypass_action_handler_ = bypass;

  // Topic handlers
  ros::NodeHandle node;

  pub_wall_  = node.advertise<sensor_msgs::PointCloud2>("/explore/PCL", 10);
  pub_lines_ = node.advertise<visualization_msgs::Marker>("/explore/HoughLines", 10);
  pub_points_= node.advertise<visualization_msgs::Marker>("/explore/points", 10);
  pub_poses_ = node.advertise<geometry_msgs::PoseArray>("/explore/poses", 10);

  tf_listener_ = new tf::TransformListener();


  // Run if bypassing actionlib
  if (bypass_action_handler_)
  {
    callbacksEnable();
    run();
  }
}


void BoxLocator::callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  current_odom_ = *odom_msg;
}


void BoxLocator::callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Convert msg to pointcloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud;

  pcl::fromROSMsg (*cloud_msg, cloud);
  input_pointcloud = cloud.makeShared();


  // Remove points that are too close or too far, or out of range
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Target roughly 40m either left, right or forward
  double laser_min_range = 30;
  double laser_max_range = 50;
  double laser_min_angle = -M_PI/2;
  double laser_max_angle = M_PI/2;


  // Compute square of ranges
  laser_min_range *= laser_min_range;
  laser_max_range *= laser_max_range;
  for (int i=0; i<input_pointcloud->points.size(); i++)
  {
    pcl::PointXYZ p = input_pointcloud->points[i];
    double r = p.x*p.x + p.y*p.y;
    if (r > laser_max_range || r < laser_min_range)
      continue;

    double angle = atan2(p.y, p.x);
    if (angle > laser_max_angle || angle < laser_min_angle)
      continue;

    cloud_filtered->points.push_back (p);
  }

  /*
  {
    sensor_msgs::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_cluster_msg);
    cloud_cluster_msg.header.frame_id = cloud_msg->header.frame_id;
    cloud_cluster_msg.header.stamp = ros::Time::now();
    pub_wall_.publish(cloud_cluster_msg);
  }
  */

  if (cloud_filtered->points.size() == 0)
  {
    std::cout << "No laser points detected nearby.\n";
    return;
  }


  // Get clusters
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector;
  getCloudClusters(cloud_filtered ,pc_vector);


  // Get size of each cluster
  std::vector<Eigen::Vector3f> dimension_list;
  std::vector<Eigen::Vector4f> centroid_list;
  std::vector<std::vector<pcl::PointXYZ> > corners_list;
  computeBoundingBox(pc_vector, dimension_list, centroid_list, corners_list);


  // Only keep the clusters that are likely to be panels
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector_clustered;
  for (int i = 0; i< dimension_list.size(); i++)
  {
    if (dimension_list[i][2] <= 1.5 && dimension_list[i][1] <= 1.5)
      pc_vector_clustered.push_back(pc_vector[i]);
  }


  if (pc_vector_clustered.size() == 0)
  {
    std::cout << "Could not find panel cluster.\n";
    //action_handler->setFailure();
    return;
  }

  // Publish cluster clouds
  for (int i=0; i<pc_vector_clustered.size(); i++)
  {
    sensor_msgs::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*pc_vector_clustered[i], cloud_cluster_msg);
    cloud_cluster_msg.header.frame_id = cloud_msg->header.frame_id;
    cloud_cluster_msg.header.stamp = ros::Time::now();
    pub_wall_.publish(cloud_cluster_msg);

    if (pc_vector_clustered.size() > 1)
      usleep(200*1000);
  }

  /* Select one cloud */
  if (pc_vector_clustered.size() > 1)
  {
    std::cout << "Found multiple panel clusters. Using the first one.\n";
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = pc_vector_clustered[0];


  // Compute waypoint
  waypoints_ = computeWaypoint(cluster_cloud, 4);

  // Publish waypoints for visualization
  waypoints_.header.frame_id = cloud_msg->header.frame_id;
  pub_poses_.publish(waypoints_);

  if(!bypass_action_handler_)
    is_done_ = true;
}


void BoxLocator::callbacksEnable()
{
  sub_velo_  = nh_.subscribe("/velodyne_points", 1, &BoxLocator::callbackVelo, this);
  sub_odom_  = nh_.subscribe("/odometry/filtered", 1, &BoxLocator::callbackOdom, this);
}


void BoxLocator::callbacksDisable()
{
  // Unsubscribe topic handlers
  sub_velo_.shutdown();
  sub_odom_.shutdown();
}

bool BoxLocator::run()
{
  ServerResult* dummy;
  run(dummy);
}

bool BoxLocator::run(ServerResult* as_result)
{
  if (as_->isPreemptRequested())
    return false;

  is_done_ = false;

  ros::Rate r(30);
  while (!is_done_)
  {
    if (!ros::ok())
      return false;

    ros::spinOnce();
    r.sleep();
  }


  if (!bypass_action_handler_)
  {
    as_result->success = true;
    as_result->waypoints = waypoints_;

    callbacksDisable();
  }
}


geometry_msgs::PoseArray  BoxLocator::computeWaypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double distance)
{
  // Wait until we have odometry data
  if (! &current_odom_) //Check if pointer is null
  {
    ros::Rate r(30);
    ros::spinOnce();
    r.sleep();
  }

  // Find centeroid
  geometry_msgs::Point center;
  center.x = 0;
  center.y = 0;

  for ( int i = 0; i < cloud->points.size(); i++ )
  {
      center.x += cloud->points[i].x;
      center.y += cloud->points[i].y;
  }
  center.x /= cloud->points.size();
  center.y /= cloud->points.size();


  // Find direction vector to centroid
  geometry_msgs::Point dir;
  double mag = sqrt(center.x*center.x + center.y*center.y);
  dir.x = center.x / mag;
  dir.y = center.y / mag;



  // Compute waypoint
  geometry_msgs::PoseArray pose_list;
  geometry_msgs::Pose p;

  double desired_length = mag - distance;

  p.position.x = dir.x * desired_length;
  p.position.y = dir.y * desired_length;

  double yaw = atan2(dir.y,dir.x);
  p.orientation = pose_conversion::getQuaternionFromYaw(yaw);
  pose_list.poses.push_back(p);


  return pose_list;
}


void BoxLocator::computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<pcl::PointXYZ> >& corners)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  Eigen::Vector3f one_dimension;

  for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator iterator = pc_vector.begin(), end = pc_vector.end(); iterator != end; ++iterator)
  {
    cloud_plane=*iterator;

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_plane, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_plane, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);



    pcl::transformPointCloud(*cloud_plane, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;

    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    // save the centroid into the centroid vector
    centroid_list.push_back(pcaCentroid);
    // save dimenstion into dimension list
    one_dimension[0] = maxPoint.x - minPoint.x;
    one_dimension[1] = maxPoint.y - minPoint.y;
    one_dimension[2] = maxPoint.z - minPoint.z;
    dimension_list.push_back(one_dimension);


    // Transform back
    Eigen::Matrix4f bboxTransform(Eigen::Matrix4f::Identity());
    bboxTransform.block<3,3>(0,0) = eigenVectorsPCA;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corners_pca (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corners_base (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_corners_pca->points.push_back(minPoint);
    cloud_corners_pca->points.push_back(maxPoint);

    for (int i=0; i<cloud_corners_pca->points.size(); i++)
    {
      cloud_corners_pca->points[i].x -= projectionTransform(0,3);
      cloud_corners_pca->points[i].y -= projectionTransform(1,3);
      cloud_corners_pca->points[i].z -= projectionTransform(2,3);
    }

    pcl::transformPointCloud(*cloud_corners_pca, *cloud_corners_base, bboxTransform);

    // Extract corners
    std::vector<pcl::PointXYZ> c;
    for (int i=0; i<cloud_corners_base->points.size(); i++)
      c.push_back(cloud_corners_base->points[i]);

    // Save list of corners
    corners.push_back(c);
  }
}


void BoxLocator::drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id)
{
  // Publish
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.type = marker_msg.POINTS;

  marker_msg.scale.x = 0.05;
  marker_msg.scale.y = 0.05;
  marker_msg.color.a = 1.0;
  marker_msg.color.g = 1.0;

  marker_msg.points = points;

  pub_points_.publish(marker_msg);
}


std::vector<double> BoxLocator::generateRange(double start, double end, double step)
{
  std::vector<double> vec;

  vec.push_back(start);

  while(1)
  {
    start += step;
    vec.push_back(start);

    if (start > end)
      break;
  }

  return vec;
}


void BoxLocator::getCloudClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.5); // 50cm - big since we're sure the panel is far from other obstacles (ie. barriers)
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_ptr);
  ec.extract (cluster_indices);

  // Get the cloud representing each cluster
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_ptr->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    pc_vector.push_back(cloud_cluster);
  }
}
