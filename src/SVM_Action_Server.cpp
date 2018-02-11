#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_vision/SVMclassifierAction.h>
#include <robot_vision/classifier.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

std::string pkgPath = ros::package::getPath("robot_vision");


class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<robot_vision::SVMclassifierAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  robot_vision::SVMclassifierFeedback feedback_;
  robot_vision::SVMclassifierResult result_;
  classifier bowlClf;
  classifier MugClf;

public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name),
    bowlClf(pkgPath+"/bin/bowl_1.xml"),
    MugClf(pkgPath+"/bin/coffee_mug_1.xml")
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  void executeCB(const robot_vision::SVMclassifierGoalConstPtr &goal)
  {
//      ROS_INFO("Goal: %f,%f",goal->order[0],goal->order[1]);
      ROS_INFO("Goal received");

      cv::Mat testArray = cv::Mat::zeros(1, 306, CV_32FC1);

      for(size_t j=0; j<306;j++){
          testArray.at<float>(0,j)= (float)goal->order[j];
      }

      std::vector<float> predBowl = bowlClf.predict(testArray);
      std::vector<float> predMug = MugClf.predict(testArray);

      feedback_.sequence.clear();
      feedback_.sequence.push_back(0);
      feedback_.sequence.push_back(1);

      result_.sequence.clear();
      result_.sequence.push_back(predBowl[0]);
      result_.sequence.push_back(predBowl[1]);
      result_.sequence.push_back(predMug[0]);
      result_.sequence.push_back(predMug[1]);
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.publishFeedback(feedback_);
      as_.setSucceeded(result_);
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "SVMActionServer");
  ROS_INFO("SVM Action server started");
  std::cout << "pkgPath:" << pkgPath << std::endl;
  FibonacciAction fibonacci("SVMAction");
  ros::spin();

  return 0;
}
