
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
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
using namespace ros;
typedef pcl::PointXYZ XYZPoint;



//This code accumulates extracted goal candidate in the global frame and set a goal for the move_base


class Listener
{
public:
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud;
  Listener(ros::NodeHandle node);
  Publisher output;
  int time_step_counter;
  void center_cloud_cb(const sensor_msgs::PointCloud::ConstPtr& inMsg);
};

//constructor
Listener::Listener(ros::NodeHandle node){

    time_step_counter=0;
    output=node.advertise<sensor_msgs::PointCloud>("goal_cloud", 10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud_dummy (new pcl::PointCloud<pcl::PointXYZ>);
    p_cloud= p_cloud_dummy;
}



void Listener::center_cloud_cb(const sensor_msgs::PointCloud::ConstPtr& inMsg)
{
     if (inMsg->points.size()>0)
     {
    	ros::Time now = ros::Time(0);
    	sensor_msgs::PointCloud cloud_in;
    	cloud_in.header.stamp = inMsg->header.stamp;
    	cloud_in.header.frame_id = inMsg->header.frame_id;
    	cloud_in.points=inMsg->points;
        tf::StampedTransform transform; 
        tf::TransformListener tf_listener; 
        ros::Duration(1).sleep();

         sensor_msgs::PointCloud global;
         
         //Tranform centroids of planes from each time stamp back to the gloab frame
         try {
        //the first string is target coordinate 
        tf_listener.transformPointCloud("/odom",now,cloud_in, "/velodyne",global);


            } 
        catch (tf2::ExtrapolationException &ex){
                   std::cout<<"No TF "<<std::endl;     
 
          }
         p_cloud->height   = 1;
         p_cloud->is_dense = true;

          for (size_t i = 0; i < global.points.size(); ++i)
       {
       //create single point
        XYZPoint p;
        p.x = global.points[i].x;
        p.y = global.points[i].y;
        p.z = global.points[i].z;

        // assign pointcloud value
        p_cloud->points.push_back(p);
        ++p_cloud->width;        
        }



       sensor_msgs::PointCloud goal_cloud;
       goal_cloud.header.stamp= ros::Time::now();
       goal_cloud.header.frame_id = "odom";

       if (p_cloud->points.size()>0)
       {
         goal_cloud.points.resize(p_cloud->points.size());

  
     
         for (size_t i = 0; i < p_cloud->points.size(); ++i)
         {
            goal_cloud.points[i].x=p_cloud->points[i].x;
            goal_cloud.points[i].y=p_cloud->points[i].y;
            goal_cloud.points[i].z=p_cloud->points[i].z;      
         }

       }
       output.publish(goal_cloud);
       // make a counter here if certain timae interval is exceeded and no cluser meet the threshold clear everything thing in the point cloud p_cloud
      
         ++time_step_counter;
         if(time_step_counter>=30)
           {
                p_cloud->points.clear(); 
                p_cloud->width=0;
                time_step_counter=0;
                  
           }
        // Clustering centroid of plane from each time stamp with pcl in the global frame

        std::vector<pcl::PointIndices> cluster_indices;
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (p_cloud);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // Clustering
        ec.setClusterTolerance (2); // 2m
        ec.setMinClusterSize (5);
        ec.setMaxClusterSize (30);
        ec.setSearchMethod (tree);
        ec.setInputCloud (p_cloud);
        ec.extract (cluster_indices);

        // check the point number of each cluster to find the most stable goal point candidate) 
          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
              {

                      if(it->indices.size()>7)
                      {
                              std::cout<<"new goal point get!"<<std::endl;       
                              // take mean over the points in the cluster
                              //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                               double point_num=0;
                               double x=0;
                               double y=0;
                               double z=0;
                               for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                               {   

                                   XYZPoint p;                   
                                   p=p_cloud->points[*pit]; //*
                                   x+=p.x;
                                   y+=p.y;
                                   z+=p.z;
                                   ++point_num;
                               }      
                               x=x/point_num;
                               y=y/point_num;
                               z=z/point_num; 
                               // set this point as the goal point
                               std::vector<double> goal_vector;
                               goal_vector.push_back(x);
                               goal_vector.push_back(y);
                               goal_vector.push_back(z);
                               ros::param::set("/panel_goal", goal_vector);
                               std::cout<<"It is"<< x<<"," <<y<<","<<z<<std::endl; 
                                p_cloud->points.clear(); 
              			p_cloud->width=0;
              			time_step_counter=0;
                               // ros::shutdown();
                      }
 
         
                        
              } 

     }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_class");
  ros::NodeHandle n;
  Listener listener(n);
  ros::Subscriber sub = n.subscribe("center_cloud",10, &Listener::center_cloud_cb, &listener);
  ros::spin();
  return 0;
}
