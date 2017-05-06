#include "ros/ros.h"
#include <iostream>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "velodyne_pointcloud/point_types.h"

#include <laser_geometry/laser_geometry.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

//#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <actionlib/server/simple_action_server.h>
#include <kuri_mbzirc_challenge_2_msgs/PanelPositionAction.h>

#include <unistd.h>

const double DEG2RAD = M_PI/180.0;
const double RAD2DEG = 1/DEG2RAD;


typedef kuri_mbzirc_challenge_2_msgs::PanelPositionAction PanelPositionAction;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionFeedback PanelPositionFeedback;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionResult PanelPositionResult;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionGoalConstPtr PanelPositionGoalConstPtr;


class PanelPositionActionHandler
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<PanelPositionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  PanelPositionFeedback feedback_;
  PanelPositionResult result_;

public:
  bool is_node_enabled;

  PanelPositionActionHandler(std::string name) :
    as_(nh_, name, boost::bind(&PanelPositionActionHandler::executeCB, this, _1), false),
    action_name_(name)
  {
    is_node_enabled = false;
    as_.start();
  }

  ~PanelPositionActionHandler(void){}

  void executeCB(const PanelPositionGoalConstPtr &goal)
  {
    is_node_enabled = true;

    ros::Rate r(30);
    while (1)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        // set the action state to preempted
        as_.setPreempted();
        break;
      }

      if (!is_node_enabled)
      {
        // set the action state to succeeded
        as_.setSucceeded(result_);
        break;
      }

      ros::spinOnce();
      r.sleep();
    }
  }

  void setSuccess(geometry_msgs::PoseArray waypoints)
  {
    is_node_enabled = false;

    result_.success = true;
    result_.waypoints = waypoints;
  }

  void setFailure()
  {
    is_node_enabled = false;

    result_.success = false;
  }


};


typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;


// ======
// Prototypes
// ======
void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<pcl::PointXYZ> >& corners);
void getCloudClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector);

void drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id);
std::vector<double> generateRange(double start, double end, double step);

void callbackScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg);


// =====
// Variables
// =====
ros::Subscriber sub_odom;
ros::Subscriber sub_scan;
ros::Publisher  pub_wall;
ros::Publisher  pub_points;
ros::Publisher  pub_poses;
tf::TransformListener *tf_listener;

PanelPositionActionHandler *action_handler;
bool bypass_action_handler = false;
std::string actionlib_topic = "get_panel_cluster";
nav_msgs::Odometry current_odom;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "circumnavigation_waypoints");
  ros::NodeHandle node;

  // Topic handlers
  sub_scan  = node.subscribe("/scan", 1, callbackScan);
  sub_odom  = node.subscribe("/odometry/filtered", 1, callbackOdom);

  action_handler = new PanelPositionActionHandler(actionlib_topic);

  pub_wall  = node.advertise<sensor_msgs::PointCloud2>("/explore/PCL", 10);
  pub_points= node.advertise<visualization_msgs::Marker>("/explore/points", 10);
  pub_poses = node.advertise<geometry_msgs::PoseArray>("/explore/poses", 10);

  tf_listener = new tf::TransformListener();

  // Parse input
  if (argc > 1)
  {
    // Enable node without waiting for actionlib calls
    bypass_action_handler = true;
    std::cout << "Bypassing actionlib\n";
  }
  else
  {
    std::cout << "Waiting for messages on the \"" << actionlib_topic << "\" actionlib topic\n";
  }

  ros::spin();
  return 0;
}

geometry_msgs::Quaternion getQuaternionFromYaw(double yaw)
{
  geometry_msgs::Quaternion quat;

  tf::Quaternion tf_q;
  tf_q = tf::createQuaternionFromYaw(yaw);

  quat.x = tf_q.getX();
  quat.y = tf_q.getY();
  quat.z = tf_q.getZ();
  quat.w = tf_q.getW();

  return quat;
}

Eigen::Matrix4d convertStampedTransform2Matrix4d(tf::StampedTransform t)
{
  // Get translation
  Eigen::Vector3d T1(
      t.getOrigin().x(),
      t.getOrigin().y(),
      t.getOrigin().z()
  );

  // Get rotation matrix
  tf::Quaternion qt = t.getRotation();
  tf::Matrix3x3 R1(qt);

  Eigen::Matrix3d R;
  tf::matrixTFToEigen(R1,R);

  // Set
  Eigen::Matrix4d tf_eigen;
  tf_eigen.setZero ();
  tf_eigen.block (0, 0, 3, 3) = R;
  tf_eigen.block (0, 3, 3, 1) = T1;
  tf_eigen (3, 3) = 1;

  return tf_eigen;
}

geometry_msgs::PoseArray computeSimpleWaypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double size)
{
  // Wait until we have odometry data
  if (! &current_odom) //Check if pointer is null
  {
    ros::Rate r(30);
    ros::spinOnce();
    r.sleep();
  }

  geometry_msgs::PoseArray pose_list;
  pcl::PointXYZ minPoint, maxPoint;
  geometry_msgs::Point center;

  double box_size = size/2;

  // Find center of bounds (rough)
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  center.x = (minPoint.x + maxPoint.x)/2;
  center.y = (minPoint.y + maxPoint.y)/2;


  // Make sure we generate waypoints in clockwise order in 3x3 grid
  const int pair_count = 8*2;
  static const int ij_pairs[] = {-1,-1,
                                -1, 0,
                                -1, 1,
                                 0, 1,
                                 1, 1,
                                 1, 0,
                                 1,-1,
                                 0,-1};

  // Compute waypoints
  std::vector<geometry_msgs::Pose> temp_poses;
  for (int m=0; m<pair_count; m+=2)
  {
    int i = ij_pairs[m];
    int j = ij_pairs[m+1];

    geometry_msgs::Pose p;
    p.position.x = center.x + box_size*i;
    p.position.y = center.y + box_size*j;

    double yaw = atan2(-j,-i);
    p.orientation = getQuaternionFromYaw(yaw);

    temp_poses.push_back(p);
  }

  // Find closest waypoint
  double min_dist = 1/.0;
  int min_dist_idx = 0;
  for (int i=0; i<temp_poses.size(); i++)
  {
    double x = temp_poses[i].position.x - current_odom.pose.pose.position.x;
    double y = temp_poses[i].position.y - current_odom.pose.pose.position.y;

    double d = x*x + y*y;

    if (d < min_dist)
    {
      min_dist = d;
      min_dist_idx = i;
    }
  }

  // Order waypoints
  for (int i=min_dist_idx; i<temp_poses.size(); i++)
    pose_list.poses.push_back( temp_poses[i] );

  for (int i=0; i<min_dist_idx; i++)
    pose_list.poses.push_back( temp_poses[i] );

  return pose_list;
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  if (!action_handler->is_node_enabled && !bypass_action_handler)
    return;

  current_odom = *odom_msg;
}

void callbackScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if (!action_handler->is_node_enabled && !bypass_action_handler)
    return;

  // Convert scan message to cloud message
  sensor_msgs::PointCloud2 cloud_msg;
  laser_geometry::LaserProjection projector_;
  projector_.projectLaser(*scan_msg, cloud_msg);
  //projector_.transformLaserScanToPointCloud("base_link", *scan_msg, cloud, *tf_listener);

  // Convert msg to pointcloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud;

  pcl::fromROSMsg (cloud_msg, cloud);
  input_pointcloud = cloud.makeShared();


  // Remove points that are too close or too far
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  double laser_min_range = 0.6; // Used to ignore the bumpers
  double laser_max_range = 5; // Used to ignore far points

  for (int i=0; i<input_pointcloud->points.size(); i++)
  {
    pcl::PointXYZ p = input_pointcloud->points[i];
    double r = sqrt(p.x*p.x + p.y*p.y);
    if (r > laser_max_range || r < laser_min_range)
      continue;

    cloud_filtered->points.push_back (p);
  }

  if (cloud_filtered->points.size() == 0)
  {
    std::cout << "No laser points detected nearby.\n";
    action_handler->setFailure();
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
    action_handler->setFailure();
    return;
  }

  if (pc_vector_clustered.size() > 1)
  {
    std::cout << "Found multiple panel clusters. Using the first one.\n";
  }

  // Publish cluster clouds
  for (int i=0; i<pc_vector_clustered.size(); i++)
  {
    sensor_msgs::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*pc_vector_clustered[i], cloud_cluster_msg);
    cloud_cluster_msg.header.frame_id = scan_msg->header.frame_id;
    cloud_cluster_msg.header.stamp = ros::Time::now();
    pub_wall.publish(cloud_cluster_msg);

    if (pc_vector_clustered.size() > 1)
      usleep(100*1000);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = pc_vector_clustered[0];


  // Transform cluster to world frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  try
  {
    tf::StampedTransform transform;
    Eigen::Matrix4d tf_eigen;

    tf_listener->lookupTransform("/odom", scan_msg->header.frame_id, ros::Time(0), transform);
    tf_eigen = convertStampedTransform2Matrix4d(transform);


    pcl::transformPointCloud(*cluster_cloud, *tf_cloud, tf_eigen);

    // Compute waypoints around object
    geometry_msgs::PoseArray waypoints;
    waypoints = computeSimpleWaypoints(tf_cloud, 5);

    // Publish waypoints
    waypoints.header.frame_id = "/odom";
    pub_poses.publish(waypoints);

    action_handler->setSuccess(waypoints);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }

}

void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<pcl::PointXYZ> >& corners)
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

void drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id)
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

std::vector<double> generateRange(double start, double end, double step)
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

void getCloudClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (1.0); // 100cm - big since we're sure the panel is far from other obstacles (ie. barriers)
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (2500);
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
