#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub;

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "tf_emitter");
  ros::NodeHandle nh;
  
  ros::Rate loop_rate(0.2);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3 (0.0,0.0,0.0));
  tf::Quaternion q;
  q.setRPY(0,0,0);
  transform.setRotation(q);

   while(ros::ok()){
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
      ROS_INFO("WORLD FRAME PUBLISHED");
      ros::spinOnce ();
      loop_rate.sleep();
   }
  // Spin
}
