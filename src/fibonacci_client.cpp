#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_vision/FibonacciAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<robot_vision::FibonacciAction> ac("fibonacci", true);
  ros::Rate r(1);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  robot_vision::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout;
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));


  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    ROS_INFO("RESULT");
    robot_vision::FibonacciResultConstPtr result  = ac.getResult();
    std::cout << typeid(result).name() << std::endl;
    std::cout << result->sequence.size() << std::endl;
    for(size_t i=0;i<result->sequence.size();i++){
        std::cout << result->sequence.at(i) << " ";
    }
    std::cout << std::endl;
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
