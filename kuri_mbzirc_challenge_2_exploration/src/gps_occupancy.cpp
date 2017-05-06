#include <kuri_mbzirc_challenge_2_exploration/gps_occupancy.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub_points_occ;

void GPSOccupancy::createGrid()
{
  if (!occ_tree_)
    throw std::logic_error("Occupancy grid not ready");

  if (!gps_filter_.isReady())
    throw std::logic_error("GPS filter not ready. Make sure to set the bounds, origin and orientation first");


}


void GPSOccupancy::getLikelyPanel(double* x_final, double* y_final)
{
  if (!isReady())
  {
    throw std::logic_error("Occupancy grid not ready");
  }

  if (occ_tree_->size() <= 0)
  {
    *x_final = 0;
    *y_final = 0;
  }

  // Find max node (starting frame)
  double max_odds = -20;
  octomap::OcTree::leaf_bbx_iterator max_iterator;


  octomap::point3d min (-80, -80, -2);
  octomap::point3d max ( 80,  80,  2);

  for(octomap::OcTree::leaf_bbx_iterator it  = occ_tree_->begin_leafs_bbx(min,max), end = occ_tree_->end_leafs_bbx();
      it!= end;
      ++it)
  {
    octomap::OcTreeNode* node = occ_tree_->search(it.getKey());

    if (node == NULL)
      continue;

    double odds = node->getLogOdds();
    if (odds > max_odds)
    {
      max_iterator = it;
      max_odds = odds;
    }
  }

  // Get XYZ position wrt starting position
  octomap::point3d p = occ_tree_->keyToCoord( max_iterator.getKey() );


  // Get GPS position
  GeoPoint g;
  gps_origin_.projectCartesianToGPS(p.x(), p.y(), &g.lat, &g.lon);


  // Get XYZ position wrt current position
  float x_unrotated, y_unrotated;
  gps_filter_.ref_gps_.projectGPSToCartesian( g.lat, g.lon, &x_unrotated, &y_unrotated );

  // Rotate point to match vehicle orientation
  *x_final = x_unrotated;
  *y_final = y_unrotated;
}


bool GPSOccupancy::isReady()
{
  // Check if filter is ready and occ_tree_ is created (ie. not NULL pointer)
  return gps_filter_.isReady() && occ_tree_;
}


void GPSOccupancy::setGpsBounds(std::vector<GeoPoint> arena_bounds)
{
  // start publisher
  ros::NodeHandle node_handle;
  pub_points_occ = node_handle.advertise<sensor_msgs::PointCloud2>("/explore/points_to_origin", 10);


  gps_filter_.setBounds( arena_bounds );
}


void GPSOccupancy::setOccupancyResolution(double res)
{
  occ_tree_ = new octomap::OcTree(res);
}


bool GPSOccupancy::setOccupancyProbHit(double prob)
{
  occ_tree_->setProbHit(prob);
}


bool GPSOccupancy::setOccupancyProbMiss(double prob)
{
  occ_tree_->setProbMiss(prob);
}


void GPSOccupancy::setRefGps(double lat, double lon)
{
  // Save the first GPS coordinates as the origin
  if (!gps_origin_.isInit())
    gps_origin_.update(lat, lon, ros::Time::now().toNSec());

  gps_filter_.setRefGPS( lat, lon, ros::Time::now().toNSec() );
}


void GPSOccupancy::setRefOrientation(geometry_msgs::Quaternion q)
{
  gps_filter_.setRefOrientation( q );
}


void GPSOccupancy::updateOccupancy(PcCloudPtr input_cloud, PcCloudPtr original_cloud, double decay_rate)
{
  if (!isReady())
  {
    throw std::logic_error("Occupancy grid not ready");
  }

  if (decay_rate > 1 || decay_rate < 0)
  {
    throw std::invalid_argument("Decay rate must be between 0 and 1, inclusive.");
  }

  octomap::point3d sensor_origin (0,0,0);
  double range = 70;

  // Transform points from current GPS to origin GPS
  PcCloud::Ptr pc_rotated (new PcCloud);
  Eigen::Matrix4d Ti = getTransfromMatrixToRef();
  pcl::transformPointCloud (*input_cloud, *pc_rotated, Ti);

  // Visualize these points
  //ros::NodeHandle node_handle;
  //ros::Publisher pub_points_occ = node_handle.advertise<sensor_msgs::PointCloud2>("/explore/points_to_origin", 10);
  PcCloud::Ptr pc_rotated_vis (new PcCloud);
  pcl::transformPointCloud (*original_cloud, *pc_rotated_vis, Ti);

  sensor_msgs::PointCloud2 cloud_cluster_msg;
  pcl::toROSMsg(*pc_rotated_vis, cloud_cluster_msg);
  cloud_cluster_msg.header.frame_id = "velodyne";
  cloud_cluster_msg.header.stamp = ros::Time::now();
  pub_points_occ.publish(cloud_cluster_msg);

  // Convert to octomap pointcloud format
  octomap::Pointcloud ocCloud;
  for (int j=0; j < pc_rotated->points.size(); j++)
  {
    ocCloud.push_back(pc_rotated->points[j].x,
                      pc_rotated->points[j].y,
                      pc_rotated->points[j].z);
  }


  // Decay existing cells
  if (occ_tree_->size() > 0)
  {
    octomap::point3d min (-80, -80, -2);
    octomap::point3d max ( 80,  80,  2);

    for(octomap::OcTree::leaf_bbx_iterator it  = occ_tree_->begin_leafs_bbx(min,max), end = occ_tree_->end_leafs_bbx();
        it!= end;
        ++it)
    {
      octomap::OcTreeNode* node = occ_tree_->search(it.getKey());

      if (node == NULL)
        continue;

      node->setLogOdds(node->getLogOdds() * decay_rate);
    }
  }


  // Compute free and occupied cells
  octomap::KeySet free_cells, occupied_cells;
  occ_tree_->computeUpdate(ocCloud, sensor_origin, free_cells, occupied_cells, range);


  // Insert data into tree using binary probabilities
  for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
  {
    occ_tree_->updateNode(*it, false);
  }

  for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
  {
    occ_tree_->updateNode(*it, true);
  }
}


Eigen::Matrix4d GPSOccupancy::getTransfromMatrixToRef()
{
  // ==Assumes reference is pointed North
  GeoPoint g = gps_filter_.getRefGPS();
  geometry_msgs::Quaternion ref_quat = gps_filter_.getRefQuaternion();

  // Get rotation from current position to starting positions
  double yaw = pose_conversion::getYawFromQuaternion(ref_quat);
  geometry_msgs::Quaternion q = pose_conversion::getQuaternionFromYaw(yaw);
  Eigen::Matrix3d R = pose_conversion::getRotationMatrix(q);

  // Get displacement
  float x, y;
  gps_origin_.projectGPSToCartesian(g.lat, g.lon, &x, &y);
  Eigen::Vector3d T(x, y, 0);

  // Create final transform
  Eigen::Matrix4d tf_eigen;
  tf_eigen.setZero ();
  tf_eigen.block (0, 0, 3, 3) = R;
  tf_eigen.block (0, 3, 3, 1) = T;
  tf_eigen (3, 3) = 1;

  return tf_eigen;
}
