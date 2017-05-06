#ifndef _panel_searching_H_
#define _panel_searching_H_

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
class panel_searching
  {
  public:
    //panel_searching();  
    //~panel_searching() {}
    void pointcloud_tree_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud,   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices);    
    void plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, int& plane_id,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector);
    void bounding_box_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list);
    void panel_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_assemble, std::vector<Eigen::Vector4f>& sifted_centroid_list);
    void assembler(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_assemble,std::vector<Eigen::Vector3f>& dimension_list,std::vector<Eigen::Vector4f>& centroid_list,std::vector<Eigen::Vector4f>& sifted_centroid_list);
    
  };
#endif
