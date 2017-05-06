#include <stdexcept>
#include <stdint.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <kuri_mbzirc_challenge_2_exploration/gps_conversion.h>
#include <kuri_mbzirc_challenge_2_exploration/pointcloud_gps_filter.h>
#include <kuri_mbzirc_challenge_2_tools/pose_conversion.h>

PointcloudGpsFilter::PointcloudGpsFilter():
  ready_mask_(0)
{

}


void PointcloudGpsFilter::filterBounds (PcCloud::Ptr cloud_out)
{
  if (!isReady())
  {
    throw std::runtime_error("PointcloudGpsFilter not ready");
  }

  bool is_visualized = false;

  // Get transformation (Yaw -> rotation matrix -> transformation matrix)
  Eigen::Matrix3d R = pose_conversion::getRotationMatrix(ref_gps_quaternion_);
  Eigen::Matrix4d Ti = pose_conversion::getTransformationMatrix(R);

  /*
  double yaw = pose_conversion::getYawFromQuaternion(ref_gps_quaternion_);
  geometry_msgs::Quaternion q = pose_conversion::getQuaternionFromYaw(yaw);
  Eigen::Matrix3d R = pose_conversion::getRotationMatrix(q);
  Eigen::Matrix4d Ti = pose_conversion::getTransformationMatrix(R);

  printf("Yaw: %lf\n", yaw);
  */

  // Orient pointcloud towards north
  PcCloud::Ptr pc_rotated (new PcCloud);
  pcl::transformPointCloud (*pc_, *pc_rotated, Ti);

  // Transform GPS bounds to cartesian
  std::vector<PcPoint> bounds_cartesian;

  for (int i=0; i < arena_bounds_.size(); i++)
  {
    GeoPoint g = arena_bounds_[i];
    PcPoint p;

    ref_gps_.projectGPSToCartesian(g.lat, g.lon, &p.x, &p.y);

    bounds_cartesian.push_back(p);
  }

  // Append bounds for visualization
  if (is_visualized)
  {
    for (int b=0; b < bounds_cartesian.size(); b++)
    {
      PcPoint bp = bounds_cartesian[b];
      for (int i=-2; i<=2; i++)
      {
        for (int j=-2; j<=2; j++)
        {
          PcPoint p;
          p.x = bp.x + i*1.0;
          p.y = bp.y + j*1.0;
          cloud_out->points.push_back(p);
        }
      }
    }
  }

  // Check which points are within bounds
  for (int i=0; i < pc_->points.size(); i++)
  {
    PcPoint p = pc_rotated->points[i];

    // Check if gps coordinates within predefined bounds
    bool isWithinBounds = pointWithinBounds(bounds_cartesian, p);
    if (isWithinBounds)
    {
      //cloud_out->points.push_back(p);
      cloud_out->points.push_back(pc_->points[i]);
    }
  }

}

GeoPoint PointcloudGpsFilter::getRefGPS()
{
  GeoPoint g;
  g.lat = ref_gps_.getLat();
  g.lon = ref_gps_.getLon();
  return g;
}

geometry_msgs::Quaternion PointcloudGpsFilter::getRefQuaternion()
{
  geometry_msgs::Quaternion q;
  q.x = ref_gps_quaternion_.x;
  q.y = ref_gps_quaternion_.y;
  q.z = ref_gps_quaternion_.z;
  q.w = ref_gps_quaternion_.w;

  return q;
}

bool PointcloudGpsFilter::isReady()
{
  return (ready_mask_ == 15);
}

bool PointcloudGpsFilter::pointWithinBounds (GeoPoint p)
{
  // Adapted from http://wiki.unity3d.com/index.php?title=PolyContainsPoint
  bool inside = false;

  int j = arena_bounds_.size()-1;

  for (int i = 0; i < arena_bounds_.size(); j = i++)
  {
    GeoPoint c1 = arena_bounds_[i];
    GeoPoint c2 = arena_bounds_[j];

    if ( ((c1.lat <= p.lat && p.lat < c2.lat) || (c2.lat <= p.lat && p.lat < c1.lat)) &&
       (p.lon < (c2.lon - c1.lon) * (p.lat - c1.lat) / (c2.lat - c1.lat) + c1.lon))
       inside = !inside;
  }

  return inside;
}

bool PointcloudGpsFilter::pointWithinBounds (std::vector<PcPoint> bounds, PcPoint p)
{
  // Adapted from http://wiki.unity3d.com/index.php?title=PolyContainsPoint
  bool inside = false;

  int j = bounds.size()-1;

  for (int i = 0; i < bounds.size(); j = i++)
  {
    PcPoint c1 = bounds[i];
    PcPoint c2 = bounds[j];

    bool intersect = ((c1.y > p.y) != c2.y > p.y) &&
                      (p.x < (c2.x - c1.x)*(p.y-c1.y)/(c2.y-c1.y) + c1.x);

    if (intersect)
      inside = !inside;
  }

  return inside;
}

std::vector<std::string> PointcloudGpsFilter::readyStrings()
{
  std::vector<std::string> vec;
  std::string str;

  str = "GPS Bounds -- ";
  if (ready_mask_ & 1)
    str += "OK";
  else
    str += "NOT READY";
  vec.push_back(str);


  str = "GPS Cloud -- ";
  if (ready_mask_ & 2)
    str += "OK";
  else
    str += "NOT READY";
  vec.push_back(str);


  str = "GPS Reference -- ";
  if (ready_mask_ & 4)
    str += "OK";
  else
    str += "NOT READY";
  vec.push_back(str);


  str = "GPS Orientation -- ";
  if (ready_mask_ & 8)
    str += "OK";
  else
    str += "NOT READY";
  vec.push_back(str);

  return vec;
}

void PointcloudGpsFilter::setBounds(std::vector<GeoPoint> bounds)
{
  arena_bounds_ = bounds;
  ready_mask_ |= 1;
}

void PointcloudGpsFilter::setCloud(PcCloud::Ptr cloud_in)
{
  pc_ = cloud_in;
  ready_mask_ |= 2;
}

void PointcloudGpsFilter::setRefGPS(double lat, double lon, uint64_t timestamp)
{
  ref_gps_.update(lat, lon, timestamp);
  ready_mask_ |= 4;
}

void PointcloudGpsFilter::setRefOrientation(geometry_msgs::Quaternion q)
{
  ref_gps_quaternion_ = q;
  ready_mask_ |= 8;
}
