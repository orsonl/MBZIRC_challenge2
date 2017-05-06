#ifndef _detection_H_
#define _detection_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "kuri_mbzirc_challenge_2_exploration/point_types.h"
#include <tf/transform_listener.h>



using namespace ros;

typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

class detection
  {
  public:

    detection(ros::NodeHandle node);
    ~detection() {}

  private:
    void convertPoints(const VPointCloud::ConstPtr &inMsg);
    Subscriber input;
    Publisher output;
    Publisher center; 
};

#endif
