#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

#include <kuri_mbzirc_challenge_2_tools/pose_conversion.h>



//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// PCL Visualization
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;

// State variables
bool is_initialized_ = false;
bool is_done_ = false;
PointCloud::Ptr pc_current_, pc_prev_;

tf::TransformListener* listener;


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp2_target");
  p->removePointCloud ("vp2_source");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp2_target", vp_2);
  p->addPointCloud (cloud_source, src_h, "vp2_source", vp_2);
}



////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    double leaf = 0.2;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloud::Ptr reg_result (new PointCloud);

  //
  // Align
  pcl::IterativeClosestPoint<PointT, PointT> reg;
  reg.setMaxCorrespondenceDistance (10.0);

  reg.setTransformationEpsilon (1e-8);
  reg.setMaximumIterations (50);
  reg.setInputSource (src);
  reg.setInputTarget (tgt);
  reg.align (*reg_result);

  // Get the transformation from source to target
  Ti = reg.getFinalTransformation ();
  //targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_src, *output, Ti);
  final_transform = Ti;
 }





void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  if (is_done_)
    return;


  // Convert msg to pointcloud
  PointCloud cloud;

  pcl::fromROSMsg (*cloud_msg, cloud);
  pc_current_ = cloud.makeShared();


  // Transform to the more stable odom frame
  tf::StampedTransform transform;
  try
  {
    listener->lookupTransform("odom", "velodyne", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  Eigen::Matrix4d Ti = pose_conversion::convertStampedTransform2Matrix4d(transform);


  // Transform cloud
  //PointCloud::Ptr output (new PointCloud);
  //pcl::transformPointCloud (*pc_current_, *output, Ti);
  pcl::transformPointCloud (*pc_current_, *pc_current_, Ti);

  if (!is_initialized_)
  {
    pc_prev_ = pc_current_;
    is_initialized_ = true;

    PCL_INFO ("Press q to begin the registration.\n");
    showCloudsLeft(pc_prev_, pc_current_);
    p-> spin();
    return;
  }


  //Filter inputs
  pcl::console::print_highlight ("Filtering z...\n");
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2, 3);
  //pass.setFilterLimitsNegative (true);
  pass.setInputCloud (pc_prev_);
  pass.filter (*pc_prev_);
  pass.setInputCloud (pc_current_);
  pass.filter (*pc_current_);



  // Align
  PointCloud::Ptr result (new PointCloud);
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;


  // Add visualization data
  showCloudsLeft(pc_prev_, pc_current_);

  PointCloud::Ptr temp (new PointCloud);
  //PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
  {
    pcl::ScopeTime t("Alignment");
    pairAlign (pc_prev_, pc_current_, temp, pairTransform, true);
  }

  showCloudsRight(pc_current_, temp);
  p->spinOnce();

  //std::cout << pairTransform << "\n";

  // Update clouds
  pc_prev_ = pc_current_;
  //is_done_ = true;

  p->spinOnce();

}



// Align a rigid object to a scene with clutter and occlusions
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_velodyne_alignment");
  ros::NodeHandle node;

  listener = new tf::TransformListener();
  ros::Subscriber sub_velo  = node.subscribe("/velodyne_points", 1, callbackVelo);

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  ros::spin();
  return 0;
}
