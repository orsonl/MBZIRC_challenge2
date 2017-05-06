#ifndef KURI_MBZIRC_CHALLENGE_2_TOOLS_GPS_OCCUPANCY_H_
#define KURI_MBZIRC_CHALLENGE_2_TOOLS_GPS_OCCUPANCY_H_

#include <iostream>
#include <stdexcept>

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <kuri_mbzirc_challenge_2_exploration/gps_conversion.h>
#include <kuri_mbzirc_challenge_2_exploration/pointcloud_gps_filter.h>
#include <kuri_mbzirc_challenge_2_tools/pose_conversion.h>

class GPSOccupancy
{
private:
  GPSHandler gps_origin_;

public:
  PointcloudGpsFilter gps_filter_;
  octomap::OcTree* occ_tree_;

  void createGrid();

  void getLikelyPanel(double* x_final, double* y_final);
  Eigen::Matrix4d getTransfromMatrixToRef();

  bool isReady();

  void setGpsBounds(std::vector<GeoPoint> arena_bounds);
  void setRefGps(double lat, double lon);
  void setRefOrientation(geometry_msgs::Quaternion q);
  void setOccupancyResolution(double res);
  bool setOccupancyProbHit(double prob);
  bool setOccupancyProbMiss(double prob);

  void updateOccupancy(PcCloudPtr input_cloud, PcCloudPtr original_cloud, double decay_rate);
};


#endif
