#include <ros/ros.h>
#include <ctime>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <kuri_mbzirc_challenge_2_exploration/pointcloud_gps_filter.h>
#include <kuri_mbzirc_challenge_2_tools/pose_conversion.h>

PointcloudGpsFilter gps_filter;
ros::Publisher pub_points;

void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Convert msg to pointcloud
  PcCloud cloud;

  pcl::fromROSMsg (*cloud_msg, cloud);
  PcCloud::Ptr pc_current = cloud.makeShared();


  // Update filter with new input
  gps_filter.setCloud(pc_current);

  // Check if filter was updated with all other values (position, orientation)
  if (!gps_filter.isReady())
  {
    printf("Not ready\n");
    return;
  }

  // Perform filtering
  clock_t begin = clock();

  PcCloud::Ptr final_cloud (new PcCloud);
  gps_filter.filterBounds(final_cloud);

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

  // Debug message
  //GeoPoint curr_gps = gps_filter.getRefGPS();
  //printf("Current GPS: %lf \t %lf \n", curr_gps.lat, curr_gps.lon);
  printf("Total points: %ld \t Time taken: %lf sec\n", final_cloud->points.size(), elapsed_secs);

  //Publish message
  sensor_msgs::PointCloud2 cloud_cluster_msg;
  pcl::toROSMsg(*final_cloud, cloud_cluster_msg);
  cloud_cluster_msg.header.frame_id = cloud_msg->header.frame_id;
  cloud_cluster_msg.header.stamp = ros::Time::now();
  pub_points.publish(cloud_cluster_msg);
}

void callbackGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_filter.setRefGPS(msg->latitude, msg->longitude, msg->header.seq);
}

void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
  gps_filter.setRefOrientation(msg->orientation);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_gps_filter_velodyne");
  ros::NodeHandle node_handle("mbzirc_ch2_exploration");

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

  gps_filter.setBounds(arena_bounds);

  // Topic handlers
  ros::Subscriber sub_gps   = node_handle.subscribe("/gps/fix", 1, callbackGPS);
  ros::Subscriber sub_imu   = node_handle.subscribe("/imu/data", 1, callbackIMU);
  ros::Subscriber sub_velo  = node_handle.subscribe("/velodyne_points", 1, callbackVelo);
  pub_points = node_handle.advertise<sensor_msgs::PointCloud2>("/explore/filtered_gps_points", 10);

  ros::spin();

  return 0;
}
