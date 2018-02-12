#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>

#include <robot_vision/visualizer.h>
#include <robot_vision/cloud_segmenter.h>
#include <robot_vision/descriptors.h>
#include <robot_vision/classifier.h>
#include <robot_vision/SVMclassifierAction.h>
#include <robot_vision/robot_vision_common.h>
#include <robot_vision/robot_vision_paramsConfig.h>
#include <robot_vision/common_constants.h>

#include <dynamic_reconfigure/server.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

ros::Publisher pub;
ros::Publisher pub_segmented;


double voxelLeafSize;
int removePlaneIterations;
double distanceThreshold;
double PercentageCloud;
double tolerance;
int minClustersSize ;
int maxClustersSize ;

//namespace patch
//{
//    template < typename T > std::string to_string( const T& n )
//    {
//        std::ostringstream stm ;
//        stm << n ;
//        return stm.str() ;
//    }
//}


void callback(robot_vision::robot_vision_paramsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: voxelLeafSize: %f \n removePlaneIterations: %d \n distanceThreshold: %f \n PercentageCloud: %f \n tolerance: %f \n minClustersSize: %d \n maxClustersSize: %d \n",
            config.voxelLeafSize, config.removePlaneIterations,
            config.distanceThreshold,
            config.PercentageCloud,
            config.tolerance,
            config.minClustersSize,config.maxClustersSize);

            voxelLeafSize= config.voxelLeafSize;
            removePlaneIterations= config.removePlaneIterations;
            distanceThreshold= config.distanceThreshold;
            PercentageCloud= config.PercentageCloud;
            tolerance= config.tolerance;
            minClustersSize= config.minClustersSize;
            maxClustersSize= config.maxClustersSize;
}


/* ------------------------------------------------------------------------
 * CALL BACK HANDLER
 */
class mycb
{
  public:
    ros::Publisher marker_pub;
    visualizer::BoundingBox bb;
    visualization_msgs::MarkerArray marker_array;
    std::vector<visualization_msgs::Marker> Marker_vector;
    actionlib::SimpleActionClient<robot_vision::SVMclassifierAction> ac;
    robot_vision::SVMclassifierResultConstPtr result;
    
    mycb(ros::NodeHandle nh);
    void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    void pushMarker();  
};

mycb::mycb(ros::NodeHandle nh): bb("world"),ac("SVMAction", true)
{
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker", 1);
}

void
mycb::cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{

      ROS_INFO("received cloud");
      pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> finalClusters;
      cloud_segmenter cs;
      processPointCloud p;

      pcl::PointXYZ minPT, maxPT;



      cs.load_cloud(inputCloud);
      cs.getVoxelFiltered(voxelLeafSize,voxelLeafSize,voxelLeafSize);
      cs.removePlanes(removePlaneIterations,distanceThreshold,PercentageCloud);
      finalClusters = cs.getClusters(tolerance,minClustersSize,maxClustersSize);
      ROS_INFO("Got Final clusters %d ", (int)finalClusters.size());

      // Publish Segmented Cloud
      sensor_msgs::PointCloud2 output;
      robot_vision_common::AssembleCloud(finalClusters,output);
      // Publish Segmented Cloud end
      pub_segmented.publish(output);

      ros::Rate r(1);


      for(size_t i=0;i<finalClusters.size();i++)
      {
          p.cloudinput(finalClusters[i]);

          ac.waitForServer(); //will wait for infinite time
          // send a goal to the action
          robot_vision::SVMclassifierGoal goal;
          pcl::PointCloud<pcl::VFHSignature308>::Ptr inputDescriptor = p.getDescriptor();
          for(size_t idx=0;idx<306;idx++)
          {
              goal.order.push_back((float)inputDescriptor->points[0].histogram[idx]);
          }
          ac.sendGoal(goal);

          bool finished_before_timeout;
          finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

          static const float arr[] = {0.0,0.0};
          std::vector<float> predBowl (arr, arr + sizeof(arr) / sizeof(arr[0]) );
          std::vector<float> predMug (arr, arr + sizeof(arr) / sizeof(arr[0]) );

          if (finished_before_timeout)
          {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
            result  = ac.getResult();

            for(size_t i=0;i<result->sequence.size();i++)
            {
                std::cout << result->sequence.at(i) << " ";
            }

          }
          else
          {
              ROS_INFO("Action did not finish before the time out.");
          }
          pcl::getMinMax3D(*finalClusters[i],minPT,maxPT);
          robot_vision_common::detectObject(minPT,maxPT,result,0.3,1.5,
                                            bb,(int)i,Marker_vector);

      } // END FOR LOOP


      pub.publish(*cs.cloud_filtered);
      pushMarker();

} //END cloud_cb

void
mycb::pushMarker()
{
    marker_array.markers.resize(Marker_vector.size());
    for(size_t i=0;i<Marker_vector.size();i++)
    {
        marker_array.markers[i]=Marker_vector[i];
    }
    marker_pub.publish(marker_array);
}
//------------------------------------------------------------------------



int
main (int argc, char** argv)
{
    ros::init (argc, argv, "obj_detector");
    ros::NodeHandle nh;
    mycb cbk(nh);
    ros::Subscriber sub = nh.subscribe ("cloud_pcd", 1, &mycb::cloud_cb,&cbk);

    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ > >("output", 1);
    pub_segmented = nh.advertise<sensor_msgs::PointCloud2>("segmented", 1);

    dynamic_reconfigure::Server<robot_vision::robot_vision_paramsConfig> server;
    dynamic_reconfigure::Server<robot_vision::robot_vision_paramsConfig>::CallbackType f;


    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin ();
}
