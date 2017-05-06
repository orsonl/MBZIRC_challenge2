#include <iostream>
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

// [@todo: modify the message types from "PanelPosition" to the appropriate message type]
#include "kuri_mbzirc_challenge_2_msgs/PanelPositionAction.h"

#include <sensor_msgs/LaserScan.h>

// [@todo: modify the message types from "PanelPosition" to the appropriate message type]
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionAction   NodeAction;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionFeedback NodeFeedback;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionResult   NodeResult;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionGoalConstPtr NodeGoalConstPtr;


class NodeActionHandler
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<NodeAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  NodeFeedback feedback_;
  NodeResult result_;

public:
  bool is_node_enabled = false;

  NodeActionHandler(std::string name) :
    as_(nh_, name, boost::bind(&NodeActionHandler::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~NodeActionHandler(void){}

  void executeCB(const NodeGoalConstPtr &goal)
  {
    is_node_enabled = true;

    ros::Rate r(30);
    while (1)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        // set the action state to preempted
        as_.setPreempted();
        break;
      }

      if (!is_node_enabled)
      {
        // set the action state to succeeded
        as_.setSucceeded(result_);
        break;
      }

      ros::spinOnce();
      r.sleep();
    }
  }

  void setSuccess(std::string output_string)
  {
    is_node_enabled = false;

    // Set the result message in case of success
    result_.data_out = output_string;
  }

  void setFailure(std::string output_string)
  {
    is_node_enabled = false;

    // Set the result message in case of failure
    result_.data_out = output_string;
  }
};




// ======
// Prototypes
// ======
void callbackScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);


// ======
// Variables
// ======
NodeActionHandler *action_handler;
ros::Subscriber sub_scan;


// ======
// Method Definitions
// ======

int main(int argc, char **argv)
{
  ros::init(argc, argv, "determine_wall");
  ros::NodeHandle node;

  // Topic handlers
  sub_velo  = node.subscribe("/scan", 1, callbackScan);
  action_handler = new NodeActionHandler("get_panel_cluster");

  ros::spin();
  return 0;
}

void callbackScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  // Don't process data unless the node is running
  if (!action_handler->is_node_enabled)
    return;

  // ... PERFORM ACTION HERE ...

  // Output result
  action_handler->setSuccess("Completed successfully");
}
