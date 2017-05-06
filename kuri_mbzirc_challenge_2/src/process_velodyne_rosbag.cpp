#include <fstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

ros::Publisher  pub_cloud;
ros::Publisher  pub_velo;

void getCloudClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pc_vector)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance (0.5); // 50cm - big since we're sure the panel is far from other obstacles (ie. barriers)
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_ptr);
  ec.extract (cluster_indices);

  // Get the cloud representing each cluster
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_ptr->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    pc_vector.push_back(cloud_cluster);
  }
}


void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<pcl::PointXYZI> >& corners)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
  Eigen::Vector3f one_dimension;

  for (std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>::const_iterator iterator = pc_vector.begin(), end = pc_vector.end(); iterator != end; ++iterator)
  {
    cloud_plane=*iterator;

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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZI>);



    pcl::transformPointCloud(*cloud_plane, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZI minPoint, maxPoint;

    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    // save the centroid into the centroid vector
    centroid_list.push_back(pcaCentroid);
    // save dimenstion into dimension list
    one_dimension[0] = maxPoint.x - minPoint.x;
    one_dimension[1] = maxPoint.y - minPoint.y;
    one_dimension[2] = maxPoint.z - minPoint.z;
    dimension_list.push_back(one_dimension);


    // Transform back
    Eigen::Matrix4f bboxTransform(Eigen::Matrix4f::Identity());
    bboxTransform.block<3,3>(0,0) = eigenVectorsPCA;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corners_pca (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corners_base (new pcl::PointCloud<pcl::PointXYZI>);

    cloud_corners_pca->points.push_back(minPoint);
    cloud_corners_pca->points.push_back(maxPoint);

    for (int i=0; i<cloud_corners_pca->points.size(); i++)
    {
      cloud_corners_pca->points[i].x -= projectionTransform(0,3);
      cloud_corners_pca->points[i].y -= projectionTransform(1,3);
      cloud_corners_pca->points[i].z -= projectionTransform(2,3);
    }

    pcl::transformPointCloud(*cloud_corners_pca, *cloud_corners_base, bboxTransform);

    // Extract corners
    std::vector<pcl::PointXYZI> c;
    for (int i=0; i<cloud_corners_base->points.size(); i++)
      c.push_back(cloud_corners_base->points[i]);

    // Save list of corners
    corners.push_back(c);
  }
}


struct tracked_cluster
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  double x, y;
  double x_start, y_start;
  double distance, distance_travelled;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_velodyne_rosbag");
  ros::NodeHandle node;

  pub_cloud = node.advertise<sensor_msgs::PointCloud2>("/velo_rosbag/points", 10);
  pub_velo = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);



  std::string bag_path;

  // Parse input
  if (argc > 1)
  {
    bag_path = argv[1];
  }
  else
  {
    std::cout << "Error: Input the name of the bag file\n";
    return 0;
  }

  // Open rosbag
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));


  // Process velodyne messages
  double max_angle = DEG2RAD(15);
  double min_angle = DEG2RAD(0);
  double max_starting_distance = 70;
  double min_starting_distance = 60;
  double tracked_distance = -1;
  double tracking_radius = 5;
  bool is_tracking = false;

  std::vector<tracked_cluster> tracked;

  std::ofstream myfile;
  myfile.open ("velodyne_points.csv");
  myfile << "Average distance, Points, Average Intensity\n";

  foreach(rosbag::MessageInstance const m, view)
  {
    if (!ros::ok())
      break;

    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // Filter out points outside a given angle
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i=0; i<cloud->points.size(); i++)
    {
      double x, y, z;
      x = cloud->points[i].x;
      y = cloud->points[i].y;
      z = cloud->points[i].z;

      double angle = atan2(y, x);
      if (angle > max_angle || angle < min_angle)
        continue;

      cloud_filtered->points.push_back(cloud->points[i]);
    }

    // Get clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pc_vector;
    getCloudClusters(cloud_filtered, pc_vector);

    // Keep clusters with certain number of points
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pc_vector_filtered;
    for (int i=0; i<pc_vector.size(); i++)
    {
      int pc_size = pc_vector[i]->points.size();
      if (pc_size > 20 || pc_size < 3)
        continue;

      pc_vector_filtered.push_back(pc_vector[i]);
    }



    // Get size of each cluster
    std::vector<Eigen::Vector3f> dimension_list;
    std::vector<Eigen::Vector4f> centroid_list;
    std::vector<std::vector<pcl::PointXYZI> > corners_list;
    computeBoundingBox(pc_vector, dimension_list, centroid_list, corners_list);

    /*
    for (int i=0; i<pc_vector.size(); i++)
    {
      double x = 0, y=0;

      for (int pi=0; pi<pc_vector[i]->points.size(); pi++)
      {
        x +=pc_vector[i]->points[pi].x;
        y +=pc_vector[i]->points[pi].y;
      }

      x /= pc_vector[i]->points.size();
      y /= pc_vector[i]->points.size();

      centroid_list[i][0] = x;
      centroid_list[i][1] = y;
    }
    */


    // Update tracked entries
    for (int pci=0; pci<pc_vector.size(); pci++)
    {
      // Only keep the clusters that are likely to be panels
      if (dimension_list[pci][2] > 1.5 || dimension_list[pci][1] > 1.5)
        continue;


      if (!is_tracking)
      {
        double x = centroid_list[pci][0];
        double y = centroid_list[pci][1];
        double r = sqrt(x*x + y*y);

        if (r < min_starting_distance || r > max_starting_distance)
          continue;
      }


      bool found = false;

      // Update previous tracked entries
      for (int ti=0; ti<tracked.size(); ti++)
      {
        // Check if we're in proximity
        double x1 = centroid_list[pci][0] - tracked[ti].x;
        double y1 = centroid_list[pci][1] - tracked[ti].y;
        double dist = sqrt(x1*x1 + y1*y1);

        if (dist > tracking_radius)
          continue;
        else
          found = true;

        // Check that the distance to velodyne has decreased within some tolerance
        double x2 = centroid_list[pci][0];
        double y2 = centroid_list[pci][1];
        double dist2 = sqrt(x2*x2 + y2*y2);

        double x3 = tracked[ti].x;
        double y3 = tracked[ti].y;
        double dist3 = sqrt(x3*x3 + y3*y3);

        if (dist3 + 0.5 < dist2)
          continue;

        // Update
        tracked[ti].cloud = pc_vector[pci];
        tracked[ti].x = centroid_list[pci][0];
        tracked[ti].y = centroid_list[pci][1];

        x1 = tracked[ti].x - tracked[ti].x_start;
        y1 = tracked[ti].y - tracked[ti].y_start;
        tracked[ti].distance_travelled = sqrt(x1*x1 + y1*y1);
        tracked[ti].distance = dist2;

        // Write to file
        if (is_tracking)
        {
          double intensity = 0;
          double count = tracked[ti].cloud->points.size();

          for (int idx=0; idx < count; idx++)
          {
            intensity += tracked[ti].cloud->points[idx].intensity;
          }
          intensity /= count;

          myfile << dist2 << ", " << count << ", " << intensity << "\n";
        }

      }

      // Add new tracked entry
      if (!found && !is_tracking)
      {
        tracked_cluster t;
        t.x = centroid_list[pci][0];
        t.y = centroid_list[pci][1];
        t.x_start = centroid_list[pci][0];
        t.y_start = centroid_list[pci][1];
        t.distance_travelled = 0;
        t.cloud = pc_vector[pci];

        tracked.push_back(t);
      }
    }

    // Filter out clusters that aren't likely to be our target
    std::vector<tracked_cluster> tracked_temp;
    for (int ti=0; ti<tracked.size(); ti++)
    {
      if (tracked[ti].distance_travelled > 5)
      {
        // Found our target
        tracked_temp.clear();
        tracked_temp.push_back(tracked[ti]);

        is_tracking = true;
        tracking_radius = 5;
        break;
      }

      tracked_temp.push_back(tracked[ti]);
    }

    tracked = tracked_temp;

    std::cout << "Found " << tracked.size() << " clusters \n";

    if (tracked.size() < 3)
    {
      // Publish velodyne cloud
      sensor_msgs::PointCloud2 velo_msg;
      velo_msg = *msg;
      velo_msg.header.stamp = ros::Time::now();
      pub_velo.publish(velo_msg);


      for (int i=0; i<tracked.size(); i++)
      {
        // Publish cloud
        sensor_msgs::PointCloud2 cloud_cluster_msg;
        pcl::toROSMsg(*tracked[i].cloud, cloud_cluster_msg);
        cloud_cluster_msg.header.frame_id = "velodyne";
        cloud_cluster_msg.header.stamp = ros::Time::now();
        pub_cloud.publish(cloud_cluster_msg);

        {
          ros::Rate r(30);
          ros::spinOnce();
          r.sleep();
        }
      }
    }



    /*

    // Publish cloud
    sensor_msgs::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_cluster_msg);
    cloud_cluster_msg.header.frame_id = "velodyne";
    cloud_cluster_msg.header.stamp = ros::Time::now();
    pub_cloud.publish(cloud_cluster_msg);

    std::cout << cloud->points.size() << " Got it \n";
    std::cout << cloud->points[0].intensity << "\n";
    */

    {
      ros::Rate r(30);
      ros::spinOnce();
      r.sleep();
    }
    //break;
  }


  myfile.close();
  bag.close();


  return 0;
}
