#ifndef POINTCLOUD_GPS_FILTER_H
#define POINTCLOUD_GPS_FILTER_H

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <kuri_mbzirc_challenge_2_exploration/gps_conversion.h>
#include <stdint.h>

typedef pcl::PointXYZ PcPoint;
typedef pcl::PointCloud<PcPoint> PcCloud;
typedef pcl::PointCloud<PcPoint>::Ptr PcCloudPtr;
typedef std::vector<PcCloudPtr> PcCloudPtrList;


class PointcloudGpsFilter
{
private:
  int ready_mask_;

  std::vector<GeoPoint> arena_bounds_;
  geometry_msgs::Quaternion ref_gps_quaternion_;
  PcCloud::Ptr pc_;

  bool pointWithinBounds (GeoPoint p);
  bool pointWithinBounds (std::vector<PcPoint> bounds, PcPoint p);

public:
  GPSHandler ref_gps_;

  PointcloudGpsFilter();
  void filterBounds(PcCloud::Ptr cloud_out);

  bool isReady();
  std::vector<std::string> readyStrings();

  GeoPoint getRefGPS();
  geometry_msgs::Quaternion getRefQuaternion();

  void setBounds(std::vector<GeoPoint> bounds);
  void setCloud(PcCloud::Ptr cloud_in);
  void setRefGPS(double lat, double lon, uint64_t timestamp);
  void setRefOrientation(geometry_msgs::Quaternion q);

  //void transformCloudToCartesian();
};



#endif // POINTCLOUD_GPS_FILTER_H
