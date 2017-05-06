#ifndef KURI_MBZIRC_CHALLENGE_2_EXPLORATION_BOX_LOCATION_H_
#define KURI_MBZIRC_CHALLENGE_2_EXPLORATION_BOX_LOCATION_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../include/kuri_mbzirc_challenge_2_exploration/pointcloud_gps_filter.h"
#include <kuri_mbzirc_challenge_2_msgs/BoxPositionAction.h>


typedef kuri_mbzirc_challenge_2_msgs::BoxPositionAction Action;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionGoal Goal;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionFeedback Feedback;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionResult Result;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionGoalConstPtr GoalConstPtr;

typedef pcl::PointXYZ PcPoint;
typedef pcl::PointCloud<PcPoint> PcCloud;
typedef pcl::PointCloud<PcPoint>::Ptr PcCloudPtr;
typedef std::vector<PcCloudPtr> PcCloudPtrList;


class Belief
{
protected:
  double logodds_;
  double max_odds_;
  double min_odds_;

public:
  Belief()
  {
    logodds_ = 0;
    max_odds_ = 5; //equivalent to 99.33%
    min_odds_ =-5; //equivalent to  0.67%
  }

  static double probability2Logodds(double p)
  {
    return log(p/(1-p));
  }

  static double logodds2Probability(double logodds)
  {
    double e;
    e = exp(logodds);
    return e/(1+e);
  }

  double getProbability()
  {
    return Belief::logodds2Probability(logodds_);
  }

  double getLogOdds(){
    return logodds_;
  }

  void setProbability(double x)
  {
    if (x > 1 || x < 0)
      throw std::invalid_argument( "Probability must be between 0 and 1" );

    logodds_ = Belief::probability2Logodds(x);
  }

  void setLogodds(double x)
  {
    logodds_ = x;
  }

  void updateLogodds(double x)
  {
    logodds_ += x;

    // Check for clamping
    if (logodds_ > max_odds_)
      logodds_ = max_odds_;
    else if (logodds_ < min_odds_)
      logodds_ = min_odds_;
  }

  void updateProbability(double x)
  {
    if (x > 1 || x < 0)
      throw std::invalid_argument( "Probability must be between 0 and 1" );

    updateLogodds( Belief::probability2Logodds(x) );
  }
};

struct BoxCluster{
  PcCloudPtr point_cloud;
  geometry_msgs::Pose pose;
  Belief confidence;
};

// ======
// Classes
// ======
class BoxPositionActionHandler
{
protected:
  bool is_initiatializing_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  Feedback feedback_;
  Result result_;

  double range_max_;
  double range_min_;
  double angle_max_;
  double angle_min_;
  double detect_new_distance_;

  // Update confidence based on: 0.5 + base*exp(-lambda*distace)
  double confidence_update_base_;\
  double confidence_update_lambda_;
  double max_match_distance_;

  std::vector<BoxCluster> cluster_list;
  PcCloudPtr pc_current_;
  PcCloudPtr pc_prev_;

  PointcloudGpsFilter gps_filter_;
public:
  ros::Subscriber sub_gps;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_odom;
  ros::Subscriber sub_velo;
  ros::Publisher  pub_wall;
  ros::Publisher  pub_lines;
  ros::Publisher  pub_points;
  tf::TransformListener *tf_listener;
  nav_msgs::Odometry current_odom;

  BoxPositionActionHandler(std::string name, std::vector<GeoPoint> bounds);
  ~BoxPositionActionHandler(){}

  // actionlib
  void executeCB(const GoalConstPtr &goal);
  void setSuccess(bool success = true);

  void callbackGPS(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg);
  void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

  void              computeBoundingBox(PcCloudPtrList& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<PcPoint> >& corners);
  PcCloudPtrList    getCloudClusters(PcCloudPtr cloud_ptr);
  void              getInitialBoxClusters();
  std::vector<geometry_msgs::Pose>   getPanelPose(PcCloudPtrList);
  PcCloudPtrList    extractBoxClusters(PcCloudPtr cloud_ptr);
  PcCloudPtr        filterCloudRangeAngle(PcCloudPtr cloud_ptr, double r_min, double r_max, double a_min = -M_PI, double a_max = M_PI);
  void              transformToFrame(PcCloudPtr cloud_in, PcCloudPtr& cloud_out, std::string frame_in, std::string frame_out);

  void drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id);
  void drawClusters(std::string frame_id);

  double computeDistance(geometry_msgs::Pose p1);
  double computeDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
};

#endif
