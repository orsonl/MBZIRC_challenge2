#include <ros/ros.h>
#include <ctime>

#include <octomap_msgs/Octomap.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <kuri_mbzirc_challenge_2_exploration/gps_occupancy.h>
#include <kuri_mbzirc_challenge_2_tools/pose_conversion.h>

typedef pcl::PointXYZ PcPoint;
typedef pcl::PointCloud<PcPoint> PcCloud;
typedef pcl::PointCloud<PcPoint>::Ptr PcCloudPtr;
typedef std::vector<PcCloudPtr> PcCloudPtrList;

ros::Publisher pub_points;
ros::Publisher pub_tree;

GPSOccupancy gps_occ;

// Parameters from YAML file
double panel_max_height, panel_min_height, panel_max_range, panel_min_range, panel_max_width, panel_min_width;
double cluster_tolerance;
int cluster_max_size, cluster_min_size;
double occupancy_decay_rate;


PcCloudPtrList getCloudClusters(PcCloudPtr cloud_ptr)
{
   PcCloudPtrList pc_vector;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PcPoint>::Ptr tree (new pcl::search::KdTree<PcPoint>);
  tree->setInputCloud (cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PcPoint> ec;
  ec.setClusterTolerance (cluster_tolerance); // Max distance between points in a cluster
  ec.setMinClusterSize (cluster_min_size);
  ec.setMaxClusterSize (cluster_max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_ptr);
  ec.extract (cluster_indices);

  // Get the cloud representing each cluster
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PcCloudPtr cloud_cluster (new PcCloud);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_ptr->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    pc_vector.push_back(cloud_cluster);
  }

  return pc_vector;
}

void computeBoundingBox(PcCloudPtrList& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<PcPoint> >& corners)
{
  PcCloudPtr cloud_plane (new PcCloud ());
  Eigen::Vector3f one_dimension;

  for (PcCloudPtrList::const_iterator iterator = pc_vector.begin(), end = pc_vector.end(); iterator != end; ++iterator)
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
    PcCloudPtr cloudPointsProjected (new PcCloud);



    pcl::transformPointCloud(*cloud_plane, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PcPoint minPoint, maxPoint;

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

    PcCloudPtr cloud_corners_pca  (new PcCloud);
    PcCloudPtr cloud_corners_base (new PcCloud);

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
    std::vector<PcPoint> c;
    for (int i=0; i<cloud_corners_base->points.size(); i++)
      c.push_back(cloud_corners_base->points[i]);

    // Save list of corners
    corners.push_back(c);
  }
}

PcCloudPtrList extractBoxClusters(PcCloudPtr cloud_ptr)
{
  PcCloudPtrList pc_vector;

  if (cloud_ptr->points.size() == 0)
    return pc_vector;

  // Get clusters
  pc_vector = getCloudClusters(cloud_ptr);

  // Get size of each cluster
  std::vector<Eigen::Vector3f> dimension_list;
  std::vector<Eigen::Vector4f> centroid_list;
  std::vector<std::vector<PcPoint> > corners_list;
  computeBoundingBox(pc_vector, dimension_list, centroid_list, corners_list);

  // Only keep the clusters that are likely to be panels
  PcCloudPtrList pc_vector_clustered;
  for (int i = 0; i< dimension_list.size(); i++)
  {
    if (  (dimension_list[i][2] >= panel_min_width || dimension_list[i][1] >= panel_min_width) // One of the two dimensions exceeds the minimum bounds
       && (dimension_list[i][2] <= panel_max_width && dimension_list[i][1] <= panel_max_width))// Both dimensions below the max
      pc_vector_clustered.push_back(pc_vector[i]);
  }

  return pc_vector_clustered;
}


void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Convert msg to pointcloud
  PcCloud cloud;

  pcl::fromROSMsg (*cloud_msg, cloud);
  PcCloud::Ptr input_cloud = cloud.makeShared();


  // Check if filter was updated with all other values (position, orientation)
  if (!gps_occ.isReady())
  {
    printf("Not ready\n");
    return;
  }


  PcCloud::Ptr processed_cloud = input_cloud;


  // Filter based on cloud bounds
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (processed_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (panel_min_height, panel_max_height);
  pass.filter (*processed_cloud);


  // Filter cloud based on gps bounds
  PcCloud::Ptr temp_cloud (new PcCloud);
  gps_occ.gps_filter_.setCloud(processed_cloud);
  gps_occ.gps_filter_.filterBounds(temp_cloud);
  processed_cloud = temp_cloud;

  // ------------------Filter cloud radially based on starting position


  // Filter to obtain panel-like objects
  PcCloudPtrList pc_vec = extractBoxClusters(processed_cloud);

    // Convert list into a single point cloud
  PcCloud::Ptr temp_cloud2 (new PcCloud);
  for (int i=0; i < pc_vec.size(); i++)
  {
    *temp_cloud2 += *pc_vec[i];
  }

  processed_cloud = temp_cloud2;


  // ------------------ Project onto X-Y plane


  // Update occupancy
  gps_occ.updateOccupancy(processed_cloud, input_cloud, occupancy_decay_rate);


  //Publish cloud
  processed_cloud->height = 1;
  processed_cloud->width = processed_cloud->points.size();

  sensor_msgs::PointCloud2 cloud_cluster_msg;
  pcl::toROSMsg(*processed_cloud, cloud_cluster_msg);
  cloud_cluster_msg.header.frame_id = cloud_msg->header.frame_id;
  cloud_cluster_msg.header.stamp = ros::Time::now();
  pub_points.publish(cloud_cluster_msg);

  // Publish occupancy
  octomap_msgs::Octomap octo_msg;
  octomap_msgs::fullMapToMsg (*gps_occ.occ_tree_, octo_msg);

  octo_msg.header.frame_id = "velodyne";
  octo_msg.header.stamp = ros::Time::now();
  pub_tree.publish(octo_msg);
}

void callbackGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_occ.setRefGps(msg->latitude, msg->longitude);
}

void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
  gps_occ.setRefOrientation(msg->orientation);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_gps_occupancy");
  ros::NodeHandle node_handle("mbzirc_ch2_exploration");


  // ===============
  // Load parameters
  // ===============
  std::vector<GeoPoint> arena_bounds;
  for (int i=1; i<=4; i++)
  {
    GeoPoint c;
    char param_name[50]; // Holds name of parameter

    sprintf(param_name, "arena_gps_bounds/corner%d/lat", i);
    node_handle.param(param_name, c.lat, 999.0);

    sprintf(param_name, "arena_gps_bounds/corner%d/lon", i);
    node_handle.param(param_name, c.lon, 999.0);

    if (c.lat == 999.0| c.lon == 999.0)
    {
      printf("Error: Parameter arena_gps_bounds/corner%d not defined. Exiting.\n", i);
      return -1;
    }

    arena_bounds.push_back(c);
  }

  node_handle.param("panel_information/min_range", panel_min_range, 0.0);
  node_handle.param("panel_information/max_range", panel_max_range, 120.0);
  node_handle.param("panel_information/min_height", panel_min_height, 0.0);
  node_handle.param("panel_information/max_height", panel_max_height, 2.0);
  node_handle.param("panel_information/min_width", panel_min_width, 0.5);
  node_handle.param("panel_information/max_width", panel_max_width, 1.5);

  node_handle.param("filter_cluster_settings/tolerance", cluster_tolerance, 1.5);
  node_handle.param("filter_cluster_settings/min_cluster_size", cluster_min_size, 3);
  node_handle.param("filter_cluster_settings/max_cluster_size", cluster_max_size, 5000);

  node_handle.param("occupancy_grid_settings/decay_rate", occupancy_decay_rate, 0.9);


  double grid_resolution, grid_prob_hit, grid_prob_miss;
  node_handle.param("occupancy_grid_settings/resolution", grid_resolution, 1.0);
  node_handle.param("occupancy_grid_settings/prob_hit", grid_prob_hit, 0.6);
  node_handle.param("occupancy_grid_settings/prob_miss", grid_prob_miss, 0.4);


  // ===============
  // Set up occupancy grid
  // ===============
  gps_occ.setOccupancyResolution(grid_resolution);
  gps_occ.setOccupancyProbHit(grid_prob_hit);
  gps_occ.setOccupancyProbMiss(grid_prob_miss);
  gps_occ.setGpsBounds(arena_bounds);

  // Feed it dummy cloud to get it "Ready"
  PcCloud::Ptr dummy_cloud (new PcCloud);
  gps_occ.gps_filter_.setCloud(dummy_cloud);


  // ===============
  // Topic handlers
  // ===============
  ros::Subscriber sub_gps   = node_handle.subscribe("/gps/fix", 1, callbackGPS);
  ros::Subscriber sub_imu   = node_handle.subscribe("/imu/data", 1, callbackIMU);
  ros::Subscriber sub_velo  = node_handle.subscribe("/velodyne_points", 1, callbackVelo);
  pub_points = node_handle.advertise<sensor_msgs::PointCloud2>("/explore/filtered_gps_points", 10);
  pub_tree   = node_handle.advertise<octomap_msgs::Octomap>("/explore/octomap", 10);

  // Every 1 second, get the most probable panel candidate and move towards it
  int freq = 30;
  int count = 1;
  ros::Rate rate (freq);

  while(ros::ok())
  {
    count++;
    count %= freq;

    if (count == 0)
    {
      double x, y;
      gps_occ.getLikelyPanel(&x, &y);
      printf("Panel x: %lf, y: %lf, r: %lf\n", x, y, sqrt(x*x + y*y));
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
