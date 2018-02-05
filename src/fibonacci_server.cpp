#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_vision/FibonacciAction.h>


class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<robot_vision::FibonacciAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  robot_vision::FibonacciFeedback feedback_;
  robot_vision::FibonacciResult result_;

public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  void executeCB(const robot_vision::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO(
                "%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",
                action_name_.c_str(),
                goal->order,
                feedback_.sequence[0],
                feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);

      // publish the feedback
      as_.publishFeedback(feedback_);
      ROS_INFO("Publishing Feedback %i",i);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}
