/***************************************************************************
 *   Copyright (C) 2016 - 2017 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Husameldin Mukhtar, VSAP  <husameldin.mukhtar@kustar.ac.ae>        *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cmath>

ros::Publisher planePub;
ros::Publisher wrenchPub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2 cloud_filtered;
  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);
 
  // Convert to ROS data type
  sensor_msgs::PointCloud2 planeOutput,wrenchOutput;
  pcl_conversions::fromPCL(cloud_filtered, planeOutput);
  //planeOutput = *input;
  //wrenchOutput = *input;

  ROS_INFO("Received Data");
    
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*inputCloud);
    
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
   // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud(inputCloud);
  seg.segment (*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  PointCloud::Ptr planePoints(new PointCloud);
  PointCloud::Ptr wrenchPoints(new PointCloud);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }
  else
  {
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "<< coefficients->values[2] << " " << coefficients->values[3] << std::endl;
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    // Extract the inliers
    extract.setInputCloud (inputCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planePoints);
    std::cerr << "PointCloud representing the planar component: " << planePoints->width * planePoints->height << " data points." << std::endl;

    pcl::toROSMsg(*planePoints, planeOutput);
    planeOutput.header.frame_id = "camera_rgb_optical_frame";
    planeOutput.header.stamp = ros::Time::now();

    //std::cerr << "depth: "<< planeOutput->fields << std::endl;
    //ROS_INFO(planeOutput.z())

    //printf ("Cloud: width = %d, height = %d\n", planePoints->width, planePoints->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, planePoints->points)
    //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    planePub.publish(planeOutput);

    //=============================================================================================
    // Plane rotation angle
    //=============================================================================================
    double normalLength = sqrt((coefficients->values[0] * coefficients->values[0]) + (coefficients->values[1] * coefficients->values[1]) + (coefficients->values[2] * coefficients->values[2]));

    pcl::PointXYZ normalXY;
    pcl::PointXYZ normalGround;
    pcl::PointXYZ result;
    normalXY.x = 0.0;
    normalXY.y = 0.0;
    normalXY.z = 1.0;

    normalGround.x = coefficients->values[0]/normalLength;
    normalGround.y = coefficients->values[1]/normalLength;
    normalGround.z = coefficients->values[2]/normalLength;

    result.x = normalGround.y * normalXY.z - normalXY.y*normalGround.z;
    result.y = normalXY.x*normalGround.z - normalGround.x*normalXY.z;
    result.z = normalGround.x*normalXY.y - normalGround.y*normalXY.x;
    double resultLenght = sqrt((result.x * result.x) + (result.y * result.y) +(result.z * result.z));
    result.x /=resultLenght;
    result.y /=resultLenght;
    result.z /=resultLenght;
    double theta = std::acos(normalGround.z/sqrt((normalGround.x* normalGround.x)+(normalGround.y*normalGround.y)+(normalGround.z*normalGround.z)));
    //std::cout << "The crossproduct is " << result.x << " "<< result.y<< " "<< result.z <<std::endl;
    //std::cout << "norm " << std::sqrt((result.x * result.x  )+(result.y * result.y)+ (result.z * result.z))<< std::endl; 

    printf ("theta = %f\n", theta*180/3.14159);

    //=============================================================================================
    // Normals of plane points (For testing. Can be removed later!)
    //=============================================================================================
    
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (planePoints);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr plane_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*plane_normals);

    //printf ("n_x = %f, n_y = %f, n_z = %f\n", plane_normals->at(100).normal[0],plane_normals->at(100).normal[1],plane_normals->at(100).normal[2]);

    // visualize normals
    //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    //viewer.setBackgroundColor (0.0, 0.0, 0.5);
    //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(planePoints, plane_normals);

    //while (!viewer.wasStopped ())
    //{
    //  viewer.spinOnce ();
    //}
    //return 0;

    //===================================================================================================

    // Extract the outliers
    extract.setInputCloud (inputCloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*wrenchPoints);
    std::cerr << "PointCloud representing the wrench component: " << wrenchPoints->width * wrenchPoints->height << " data points." << std::endl;

    pcl::toROSMsg(*wrenchPoints, wrenchOutput);
    wrenchOutput.header.frame_id = "camera_rgb_optical_frame";
    wrenchOutput.header.stamp = ros::Time::now();

    wrenchPub.publish(wrenchOutput);
    /*
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
      std::cerr << inliers->indices[i] << "    " << inputCloud->points[inliers->indices[i]].x << " "<< inputCloud->points[inliers->indices[i]].y << " "<< inputCloud->points[inliers->indices[i]].z << std::endl;
    }
    */
  }

}



int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "panel_orientation");
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("point_cloud");;
  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  //ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
  planePub      = nh.advertise<sensor_msgs::PointCloud2>("/plane_points", 1);
  wrenchPub     = nh.advertise<sensor_msgs::PointCloud2>("/wrench_points", 1);

  // Spin
  ros::spin ();
}
