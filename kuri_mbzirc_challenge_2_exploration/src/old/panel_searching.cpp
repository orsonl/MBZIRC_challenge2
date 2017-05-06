#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>
#include "kuri_mbzirc_challenge_2_exploration/panel_searching.h"


// Cluster the input pointcloud(raw input)
void panel_searching::pointcloud_tree_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices)
{
  std::cout << "PointCloud before filtering has: " << input_pointcloud->points.size () << " data points." << std::endl; //*


  // Create the filtering object: downsample the dataset using a leaf size of 1cm

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (input_pointcloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  // If the cluster tolerance is too small the algorithm will fail to find anything or divid one object into multiple laysers
  // Since the Velodyne has a lower resolution between layers
  ec.setClusterTolerance (0.2); // 20cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  std::cout<<"cluster no. :"<<cluster_indices.size()<<std::endl;
}



// Fit a plane model to each cluster
void panel_searching::plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, int& plane_id,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new      pcl::PointCloud<pcl::PointXYZ>);
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  // vertial plane fitting
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE); 
  seg.setAxis(Eigen::Vector3f(0,0,1));
  seg.setEpsAngle (0.4);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
  int id=0;
  int nr_points = (int) input_pointcloud->points.size ();
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> local_pc_vector;
  while (input_pointcloud->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (input_pointcloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input_pointcloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl; 
    ++plane_id;
    // save the pc in the local pc vector
    local_pc_vector.push_back(cloud_plane);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *input_pointcloud = *cloud_f;
  }


  if (local_pc_vector.size()<3)
  {
    std::cout << "plane size"<<std::endl;
    std::cout << local_pc_vector.size()<<std::endl;
    std::cout << "plane appended"<<std::endl;
    pc_vector.insert(pc_vector.end(), local_pc_vector.begin(), local_pc_vector.end());

  }
}


//Find the bounding box for each planar point cloud in order to identify the: Dimension, centroid, and density.
void panel_searching::bounding_box_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;


 int id=0;


  // initiate vectors for centroids, dimensions, and densities
  //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector;

std::vector<double> density_list;
Eigen::Vector3f one_dimension;
 for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator iterator = pc_vector.begin(), end = pc_vector.end(); iterator != end; ++iterator) 
  {
     cloud_plane=*iterator; 
    ++id;
    //detect bounding box
    std::cout<<"Start finding bounding box"<<std::endl;
    // find bounding box
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
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/  watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    // save the centroid into the centroid vector
    centroid_list.push_back(pcaCentroid);
    // save dimenstion into dimension list
    one_dimension(0,0)=maxPoint.x - minPoint.x;
    one_dimension(1,0)=maxPoint.y - minPoint.y;
    one_dimension(2,0)=maxPoint.z - minPoint.z;
    dimension_list.push_back(one_dimension);
    // save density into density list
    density_list.push_back(cloud_plane->points.size()/(one_dimension(0,0)*one_dimension(1,0)*one_dimension(2,0)));
  }
}

//Decide if a planar pointcloud could be the goal of not with the following criteria:
//1. Dimension in x and y. 2. Ratio of x and y
void panel_searching::assembler(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_assemble,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list,std::vector<Eigen::Vector4f>& sifted_centroid_list)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  Eigen::Vector3f one_dimension;
  Eigen::Vector4f one_centroid;
  int idx=0;
  int pass_count=0;

  for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator iterator = pc_vector.begin(), end = pc_vector.end(); iterator != end; ++iterator)
  {
    cloud_plane=*iterator;
    //filtered with plane dimension
    one_dimension=dimension_list[idx];
    one_centroid=centroid_list[idx];
    std::cout<<one_dimension<<std::endl;
    std::cout<<"inside"<<std::endl;
    std::cout<<one_dimension[0,1]<<std::endl;
    std::cout<<one_dimension[0,2]<<std::endl;
    if( one_dimension[0,1]<1.5  && one_dimension[0,2]<1.5 && one_dimension[0,2]/one_dimension[0,1]<2&&one_dimension[0,1]>0.5)
    {
      //append centroid list to the sifted_centroid_list

      sifted_centroid_list.push_back(one_centroid);
      //append points from single plane to the output pointcloud
      for (size_t i = 0; i < cloud_plane->points.size(); ++i)
      {
        //create single point
        pcl::PointXYZ p;
        p.x = cloud_plane->points[i].x;
        p.y = cloud_plane->points[i].y;
        p.z = cloud_plane->points[i].z;
        //assign pointer value
        plane_assemble->points.push_back(p);
        ++plane_assemble->width;
      }
      ++idx;
    }
  }
}






// Main function. The whole workflow is:
//1. Cluster the input raw pointcloud data. 2. Fit vertical plane to each cluster. 3. Find bounding box for each planar pointcloud. 4. Decide if a planar pointcloud could be the goal of not with pre-defined criteria
void panel_searching::panel_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_assemble,std::vector<Eigen::Vector4f>& sifted_centroid_list)
{
  pcl::PCDWriter writer;
  //create a list of point cloud for bounding box detection
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector;


  int plane_id=0;
  int j = 0;
  int id=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (input_pointcloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    id++;
    j++;
    // fit planer 
    plane_fitting(cloud_cluster, plane_id,pc_vector);    
  } 
  // find bounding box
  std::vector<Eigen::Vector3f> dimension_list;
  std::vector<Eigen::Vector4f> centroid_list;
  bounding_box_detection(input_pointcloud, pc_vector, dimension_list, centroid_list); 
  Eigen::Vector3f one_dimension;
  Eigen::Vector4f one_centroid;
  one_centroid=centroid_list[0];
  assembler(pc_vector, plane_assemble,dimension_list, centroid_list, sifted_centroid_list );
}

