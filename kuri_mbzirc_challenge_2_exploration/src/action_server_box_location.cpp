#include <kuri_mbzirc_challenge_2_exploration/box_location.h>
//#include <kuri_mbzirc_challenge_2_msgs/BoxPositionAction.h>

class BoxPositionActionHandler
{
protected:
  float progressCount;

  BoxLocator *box_locator_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ServerAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  ServerGoalConstPtr goal_;
  ServerFeedback feedback_;
  ServerResult result_;


public:
  BoxPositionActionHandler(std::string name, bool bypass) :
    as_(nh_, name, false),
    action_name_(name)
  {
    // Create main object
    box_locator_ = new BoxLocator(&as_, bypass);

    if (!bypass)
    {
      // Register the goal and feeback callbacks
      ROS_INFO("Registering callbacks for action %s", action_name_.c_str());
      as_.registerPreemptCallback(boost::bind(&BoxPositionActionHandler::preemptCB, this));
      as_.registerGoalCallback(boost::bind(&BoxPositionActionHandler::goalCB, this));

      // Start action server
      ROS_INFO("Starting server for action %s", action_name_.c_str());
      as_.start();
    }
  }

  ~BoxPositionActionHandler(){}

  void goalCB()
  {
    ROS_INFO("Accepting Goal for action %s", action_name_.c_str());
    goal_ = as_.acceptNewGoal();

    ROS_INFO("Started panel detection");
    progressCount = 0;

    // Run processing code
    bool success = box_locator_->run(&result_);

    // Output result if not preempted
    if (success)
      as_.setSucceeded(result_);
    else
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }


  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Object_Mapping");

    // Parse inputs
    bool bypass_action_handler = false;
    if (argc > 1)
    {
      // Enable node without waiting for actionlib calls
      bypass_action_handler = true;
    }

    // Initialize server
    BoxPositionActionHandler server("get_box_cluster", bypass_action_handler);
    ros::spin();
    return 0;
}
