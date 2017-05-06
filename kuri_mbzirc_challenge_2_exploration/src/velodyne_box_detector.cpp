#include "ros/ros.h"
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <velodyne_pointcloud/point_types.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <actionlib/server/simple_action_server.h>


#include <kuri_mbzirc_challenge_2_msgs/BoxPositionAction.h>
#include "../include/kuri_mbzirc_challenge_2_exploration/velodyne_box_detector.h"

#include <kuri_mbzirc_challenge_2_tools/pose_conversion.h>

#include <unistd.h>



BoxPositionActionHandler::BoxPositionActionHandler(std::string name, std::vector<GeoPoint> bounds) :
  as_(nh_, name, boost::bind(&BoxPositionActionHandler::executeCB, this, _1), false)
{
  action_name_ = name;
  is_initiatializing_ = false;

  detect_new_distance_ = 20; //Look for new clusters past this range

  // Selected so that total confidence is 0.8 @ 5m and 0.55 @ 60m
  confidence_update_base_ = 0.353;
  confidence_update_lambda_ = 0.0325;
  max_match_distance_ = 3.0;

  // Set up GPS filter
  gps_filter_.setBounds(bounds);

  // Topic handlers
  pub_wall  = nh_.advertise<sensor_msgs::PointCloud2>("/explore/PCL", 10);
  pub_lines = nh_.advertise<visualization_msgs::Marker>("/explore/HoughLines", 10);
  pub_points= nh_.advertise<visualization_msgs::Marker>("/explore/points", 10);

  tf_listener = new tf::TransformListener();

  as_.start();
}

void BoxPositionActionHandler::executeCB(const GoalConstPtr &goal)
{
  if (goal->request == goal->REQUEST_START)
  {
    // Check inputs
    if (goal->range_max < goal->range_min)
    {
      printf("Error initializing minumum and maximum ranges (%f to %f)", goal->range_min, goal->range_max);
      setSuccess(false);
      return;
    }

    if (goal->angle_max < goal->angle_min)
    {
      printf("Error initializing minumum and maximum ranges (%f to %f)", goal->angle_min, goal->angle_max);
      setSuccess(false);
      return;
    }

    range_max_ = goal->range_max;
    range_min_ = goal->range_min;
    angle_max_ = goal->angle_max;
    angle_min_ = goal->angle_min;

    // Enable node
    is_initiatializing_ = true;

    // Enable callbacks
    sub_velo  = nh_.subscribe("/velodyne_points", 1, &BoxPositionActionHandler::callbackVelo, this);
    sub_odom  = nh_.subscribe("/odometry/filtered", 1, &BoxPositionActionHandler::callbackOdom, this);
    sub_gps  = nh_.subscribe("/gps/fix", 1, &BoxPositionActionHandler::callbackGPS, this);
    sub_imu  = nh_.subscribe("/imu/data", 1, &BoxPositionActionHandler::callbackIMU, this);

    setSuccess(true);
  }

  else if (goal->request == goal->REQUEST_STOP)
  {
    // Unsubscribe topic handlers
    sub_velo.shutdown();
    sub_odom.shutdown();

    // Set return message
    setSuccess(true);
  }

  else if(goal->request == goal->REQUEST_QUERY)
  {
    // set the action state to succeeded
    setSuccess(true);
  }

  else
  {
    setSuccess(false);
  }

}

void BoxPositionActionHandler::setSuccess(bool success)
{
  result_.success = success;
  as_.setSucceeded(result_);
}



void   BoxPositionActionHandler::callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  current_odom = *odom_msg;
}


void   BoxPositionActionHandler::callbackGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_filter_.setRefGPS(msg->latitude, msg->longitude, msg->header.seq);
}


void   BoxPositionActionHandler::callbackIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
  gps_filter_.setRefOrientation(msg->orientation);
}


void   BoxPositionActionHandler::callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Convert msg to pointcloud
  PcCloud cloud;

  pcl::fromROSMsg (*cloud_msg, cloud);
  pc_current_ = cloud.makeShared();

  // ============
  // Perform GPS bounds filtering
  // ============
  gps_filter_.setCloud(pc_current_);

  // Check if filter was updated with all other values (position, orientation)
  if (!gps_filter_.isReady())
  {
    printf("GPS filter not ready\n");
    return;
  }

  PcCloud::Ptr final_cloud (new PcCloud);
  gps_filter_.filterBounds(final_cloud);


  // =============
  // Get Clusters
  // =============
  // Transform to odom frame
  transformToFrame(final_cloud, final_cloud, cloud_msg->header.frame_id, "odom");

  if (is_initiatializing_)
  {
    getInitialBoxClusters();
    drawClusters("odom");

    return;
  }


  PcCloudPtr cloud_filtered = filterCloudRangeAngle(pc_current_, range_min_, range_max_, angle_min_, angle_max_);
  PcCloudPtrList pc_vector = extractBoxClusters(cloud_filtered);
  std::vector<geometry_msgs::Pose> poses = getPanelPose(pc_vector);

  // Create vector to track updates
  /*
  std::vector<bool> is_updated_prev;
  for (int ib=0; ib < pc_vector.size(); ib++)
    is_updated_prev.push_back(false);
  */

  int pc_size = pc_vector.size();
  std::vector<bool> is_inserted_current;
  for (int ib=0; ib < pc_size; ib++)
  {
    is_inserted_current.push_back(false);
  }


  //
  // Match previous data with current data
  //
  for (int i_prev=0; i_prev<cluster_list.size(); i_prev++)
  {
    BoxCluster b1 = cluster_list[i_prev];

    // Find closest box
    double r_min = 1/.0;
    int idx = -1;

    for (int i_curr=0; i_curr<pc_vector.size(); i_curr++)
    {
      BoxCluster b2;
      b2.point_cloud = pc_vector[i_curr];
      b2.pose = poses[i_curr];
      b2.confidence.setProbability(0.5);

      double r = computeDistance(b1.pose, b2.pose);
      if (r < r_min && r < max_match_distance_)
      {
        r_min = r;
        idx = i_curr;
      }
    }

    // Check matching
    if (idx > -1)
    {
      // Match found, update it
      cluster_list[i_prev].point_cloud = pc_vector[idx];
      cluster_list[i_prev].pose = poses[idx];

      double dist = computeDistance(cluster_list[idx].pose); //distance from origin
      double p = 0.5 + confidence_update_base_*exp(-confidence_update_lambda_*dist);

      cluster_list[i_prev].confidence.updateProbability(p);
      is_inserted_current[idx] = true;
    }


    else
    {
      // Match not found. Decrease confidence
      // Highly confident there is nothing if it's not detected up close
      // Less change if there are few targets left
      double weight = 1;
      if (cluster_list.size() < 5)
        weight = double(cluster_list.size())/15;

      double dist = computeDistance(b1.pose); //distance from origin
      double p = 0.5 - weight*confidence_update_base_*exp(-confidence_update_lambda_*dist);

      cluster_list[i_prev].confidence.updateProbability(p);
    }
  }


  /*
  //
  // If all the new data is not accounted for, add it to the list
  //
  for (int i_curr=0; i_curr < pc_vector.size(); i_curr++)
  {
    if (is_inserted_current[i_curr])
      continue;

    BoxCluster b1;
    b1.point_cloud = pc_vector[i_curr];
    b1.pose = poses[i_curr];
    b1.confidence.setProbability(0.5);

    cluster_list.push_back(b1);
  }
  */



  // Delete any entries below a threshold
  int i=0;
  while ( i < cluster_list.size() )
  {
      double p = cluster_list[i].confidence.getProbability();
      if ( p < 0.3 || !std::isfinite(p) )
      {
          cluster_list.erase( cluster_list.begin() + i );
      } else
      {
          i++;
      }
  }

  // Display clouds
  drawClusters("odom");

  pc_prev_ = pc_current_;
}


double BoxPositionActionHandler::computeDistance(geometry_msgs::Pose p1)
{
  double x = p1.position.x, y = p1.position.y, z = p1.position.z;
  return sqrt(x*x + y*y + z*z);
}


double BoxPositionActionHandler::computeDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  double x = p1.position.x - p2.position.x;
  double y = p1.position.y - p2.position.y;
  double z = p1.position.z - p2.position.z;
  return sqrt(x*x + y*y + z*z);
}


void   BoxPositionActionHandler::computeBoundingBox(PcCloudPtrList& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<PcPoint> >& corners)
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


void   BoxPositionActionHandler::drawClusters(std::string frame_id)
{
  PcCloud final_cloud;

  printf("\nCluster | Points | Distance |  Angle  | Confidence\n");
  for (int i=0; i<cluster_list.size(); i++)
  {
    BoxCluster b = cluster_list[i];
    final_cloud += *b.point_cloud;

    printf("  %3d     %4lu    \t%2.1f \t%4.1f \t%2.1f\n",
           i,
           b.point_cloud->points.size(),
           computeDistance(b.pose),
           RAD2DEG( atan2(-b.pose.position.y, b.pose.position.x) ),
           b.confidence.getProbability()*100);
  }

  //Publish message

  sensor_msgs::PointCloud2 cloud_cluster_msg;
  pcl::toROSMsg(final_cloud, cloud_cluster_msg);
  cloud_cluster_msg.header.frame_id = frame_id;
  cloud_cluster_msg.header.stamp = ros::Time::now();
  pub_wall.publish(cloud_cluster_msg);
}


void   BoxPositionActionHandler::drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id)
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

  pub_points.publish(marker_msg);
}


PcCloudPtrList BoxPositionActionHandler::extractBoxClusters(PcCloudPtr cloud_ptr)
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
    if (dimension_list[i][2] <= 1.5 && dimension_list[i][1] <= 1.5)
      pc_vector_clustered.push_back(pc_vector[i]);
  }

  return pc_vector_clustered;
}


PcCloudPtr     BoxPositionActionHandler::filterCloudRangeAngle(PcCloudPtr cloud_ptr, double r_min, double r_max, double a_min, double a_max)
{
  double laser_min_range = r_min * r_min;
  double laser_max_range = r_max * r_max;
  double laser_min_angle = a_min;
  double laser_max_angle = a_max;

  bool check_angle = true;
  if (a_max >= M_PI && a_min <= -M_PI)
    check_angle = false;

  // Filter out points that are too close or too far, or out of range
  PcCloudPtr cloud_filtered (new PcCloud);

  for (int i=0; i < cloud_ptr->points.size(); i++)
  {
    PcPoint p = cloud_ptr->points[i];

    // Ignore points high above velodyne or on the floor
    if (p.z > 1.8 || p.z < 0.5)
    {
      continue;
    }

    double r = p.x*p.x + p.y*p.y;
    if (r > laser_max_range || r < laser_min_range)
      continue;

    if (check_angle)
    {
      double angle = atan2(p.y, p.x);
      if (angle > laser_max_angle || angle < laser_min_angle)
        continue;
    }

    cloud_filtered->points.push_back (p);
  }

  return cloud_filtered;
}


PcCloudPtrList BoxPositionActionHandler::getCloudClusters(PcCloudPtr cloud_ptr)
{
   PcCloudPtrList pc_vector;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PcPoint>::Ptr tree (new pcl::search::KdTree<PcPoint>);
  tree->setInputCloud (cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PcPoint> ec;
  ec.setClusterTolerance (1.5); // up to 150cm btw points - big since we're sure the panel is far from other obstacles
  ec.setMinClusterSize (3);     // at least 3 points
  ec.setMaxClusterSize (5000);
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


void BoxPositionActionHandler::getInitialBoxClusters()
{
  // >>>>>>>>>
  // Initialize
  // >>>>>>>>>
  cluster_list.clear();

  if (range_max_ == 0 && range_max_ == 0)
  {
    range_max_ = 60.0;
    range_min_ = 1.0;
  }

  if (angle_max_ == 0 && angle_min_ == 0)
  {
    angle_max_ = M_PI;
    angle_min_ = -M_PI;
  }

  PcCloudPtr cloud_filtered = filterCloudRangeAngle(pc_current_, range_min_, range_max_, angle_min_, angle_max_);

  PcCloudPtrList pc_vector = extractBoxClusters(cloud_filtered);
  std::vector<geometry_msgs::Pose> poses = getPanelPose(pc_vector);

  for (int i=0; i<pc_vector.size(); i++)
  {
    BoxCluster b;
    b.point_cloud = pc_vector[i];
    b.pose = poses[i];
    b.confidence.setProbability(0.5);

    cluster_list.push_back(b);
  }


  is_initiatializing_ = false;
}


std::vector<geometry_msgs::Pose> BoxPositionActionHandler::getPanelPose(PcCloudPtrList clusters)
{
  std::vector<geometry_msgs::Pose> poses;

  // Compute centroid of each box
  for (int ic=0; ic<clusters.size(); ic++)
  {
    PcCloudPtr pc = clusters[ic];
    geometry_msgs::Pose p;

    double pc_size = pc->points.size();

    for (int ip=0; ip < pc_size; ip++)
    {
      PcPoint pt = pc->points[ip];
      p.position.x += pt.x;
      p.position.y += pt.y;
      p.position.z += pt.z;
    }

    p.position.x /= pc_size;
    p.position.y /= pc_size;
    p.position.z /= pc_size;

    poses.push_back(p);
  }


  return poses;
}


void BoxPositionActionHandler::transformToFrame(PcCloudPtr cloud_in, PcCloudPtr& cloud_out, std::string frame_in, std::string frame_out)
{
  // Transform to the more stable odom frame
  tf::StampedTransform transform;
  try
  {
    tf_listener->lookupTransform(frame_out, frame_in, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  Eigen::Matrix4d Ti = pose_conversion::convertStampedTransform2Matrix4d(transform);


  // Transform cloud
  pcl::transformPointCloud (*cloud_in, *cloud_out, Ti);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "box_location");
  ros::NodeHandle node_handle ("mbzirc_ch2_exploration");


  // Load parameters
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

  // Action server
  BoxPositionActionHandler *action_handler;
  std::string actionlib_topic = "get_box_cluster";
  action_handler = new BoxPositionActionHandler(actionlib_topic, arena_bounds);

  std::cout << "Waiting for messages on the \"" << actionlib_topic << "\" actionlib topic\n";

  ros::spin();
  return 0;
}





