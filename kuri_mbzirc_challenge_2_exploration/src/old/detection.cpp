#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "kuri_mbzirc_challenge_2_exploration/detection.h"
#include "kuri_mbzirc_challenge_2_exploration/panel_searching.h"
#include "kuri_mbzirc_challenge_2_exploration/point_types.h"
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
//#include <tf/transform_listener.h>
//#include "kuri_mbzirc_challenge_2_exploration/transform_listener.h"

//#include <velodyne_pointcloud/point_types.h>
//#include <chrono>
//#include <thread>
#include <iostream> 
#include <stdio.h>
using namespace std;

  // Two struct from PCL not ros 
  typedef pcl::PointXYZ XYZPoint;
  typedef pcl::PointCloud<XYZPoint> XYZPointCloud;

  detection::detection(ros::NodeHandle node)
  {
    // subscribe to VelodyneScan packets
    input =
      node.subscribe("velodyne_points", 10, &detection::convertPoints, this, ros::TransportHints().tcpNoDelay(true));

    //
    output =
      node.advertise<sensor_msgs::PointCloud2>("panel_cloud", 10);
    center =
      node.advertise<sensor_msgs::PointCloud>("center_cloud", 10);
    //tf::TransformListener tf_listener; 
    //tf_listener
    //tf_listener.lookupTransform("/velodyne","/bask_link",ros::Time(0),transform);
  }


void detection::convertPoints(const VPointCloud::ConstPtr &inMsg)
{
    if (output.getNumSubscribers() == 0)         // no one listening?
      return;                                     // do nothing 
    //create pointer    
    XYZPointCloud::Ptr outMsg(new XYZPointCloud());
    outMsg->header.stamp = inMsg->header.stamp;
    outMsg->header.frame_id = inMsg->header.frame_id;
    outMsg->height = 1;
    //create pointcloud
    XYZPointCloud p_cloud;
    p_cloud.height   = 1;
    p_cloud.is_dense = true;
    char buffer [50];
    //assign pointer and pointcloud point by point from inMsg
    for (size_t i = 0; i < inMsg->points.size(); ++i)
      {
       //create single point
        XYZPoint p;
        p.x = inMsg->points[i].x;
        p.y = inMsg->points[i].y;
        p.z = inMsg->points[i].z;
       //assign pointer value
        outMsg->points.push_back(p);
        ++outMsg->width;
        // assign pointcloud value
        p_cloud.points.push_back(p);
        ++p_cloud.width;        
      }
     // port. 1   unprocessed data
     //output.publish(outMsg);

     //extract panel
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     cloud->header.stamp = inMsg->header.stamp;
     cloud->header.frame_id = inMsg->header.frame_id;
     cloud->height = 1;
     cloud->width = outMsg->width; 
     cloud->points=p_cloud.points;
     //  *cloud=p_cloud;
     // port. 2   second unprocessed data
     //output.publish(cloud); 
     // panel searching setup
     panel_searching P_S;
     std::vector<pcl::PointIndices> cluster_indices;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
     // cluster the noisy point cloud
     P_S.pointcloud_tree_clustering(cloud, cloud_filtered, cluster_indices);
     // port. 3   clustered data
     //cloud->width = cloud_filtered->points.size (); 
     //cloud->points=cloud_filtered->points;
     //output.publish(cloud); 
     XYZPointCloud::Ptr plane_assemble(new XYZPointCloud());
     std::vector<Eigen::Vector4f> sifted_centroid_list;
     plane_assemble->header.stamp = inMsg->header.stamp;
     plane_assemble->header.frame_id = inMsg->header.frame_id;
     plane_assemble->height = 1;
     P_S.panel_extraction(cloud_filtered, cluster_indices, plane_assemble, sifted_centroid_list);
      std::cout << "plane_assemble has: " << plane_assemble->points.size ()  << " data points." << std::endl; //*
     // Publish plane pointcloud for visualization 
     output.publish(plane_assemble);
     

    
     Eigen::Vector4f one_centroid;
     sensor_msgs::PointCloud center_cloud;
     //center_cloud.header.stamp= inMsg->header.stamp;
     center_cloud.header.stamp= ros::Time::now();
     center_cloud.header.frame_id = inMsg->header.frame_id;




     std::cout<<"sifted_centroid_list size is:"<<std::endl;
     std::cout<<sifted_centroid_list.size()<<std::endl;
     
     if (sifted_centroid_list.size()>0)
     {
         center_cloud.points.resize(sifted_centroid_list.size());

  
     one_centroid=sifted_centroid_list[0];
     std::cout<<"one_centroid is:"<<std::endl;
     std::cout<<one_centroid<<std::endl;


     std::cout<<one_centroid[0,0]<<std::endl;
  std::cout<<one_centroid[0,1]<<std::endl;
  std::cout<<one_centroid[0,2]<<std::endl;




     for (size_t i = 0; i < sifted_centroid_list.size(); ++i)
      {
         one_centroid=sifted_centroid_list[i];
         center_cloud.points[i].x=one_centroid[0,0];
         center_cloud.points[i].y=one_centroid[0,1];
         center_cloud.points[i].z=one_centroid[0,2];

     }
     }
     center.publish(center_cloud);
     




// Publish goal points 
     
     // Procedure
     
     //tf::StampedTransform transform; 
     //1. Transform each centroid: [x, y ,z] back to the global frame 
     //tf::TransformListener listener;

    // tf_listener.lookupTransform("/velodyne","/bask_link",ros::Time(0),transform);
     //std::cout << transform << std::endl; 


     //2. If not in the area of interest delect it 

     
     //3. Some kind of clustering  
 

     //4. Accumulate score 

   


     //5. Send the Target with the higest score to the move_base 



     
}

