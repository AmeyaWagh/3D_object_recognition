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

#include <robot_vision/pointcloudVector.h>

ros::Publisher pub;
ros::Publisher pub_segmented;
ros::Publisher pub_pcVector;


double voxelLeafSize;
int removePlaneIterations;
double distanceThreshold;
double PercentageCloud;
double tolerance;
int minClustersSize ;
int maxClustersSize ;



void reconfigCallback(robot_vision::robot_vision_paramsConfig &config, uint32_t level) 
{
  voxelLeafSize= config.voxelLeafSize;
  removePlaneIterations= config.removePlaneIterations;
  distanceThreshold= config.distanceThreshold;
  PercentageCloud= config.PercentageCloud;
  tolerance= config.tolerance;
  minClustersSize= config.minClustersSize;
  maxClustersSize= config.maxClustersSize;

  ROS_INFO("------------------------------------------------------------------");
  ROS_INFO("Reconfigure Request:");
  ROS_INFO("VoxelLeafSize: %f",config.voxelLeafSize);
  ROS_INFO("RemovePlaneIterations: %d",config.removePlaneIterations);
  ROS_INFO("DistanceThreshold: %f",config.distanceThreshold);
  ROS_INFO("PercentageCloud: %f",config.PercentageCloud);
  ROS_INFO("Tolerance: %f",config.tolerance);
  ROS_INFO("(minClustersSize: %d)(maxClustersSize: %d)",config.minClustersSize,config.maxClustersSize);
  ROS_INFO("------------------------------------------------------------------\n");
}


/* ------------------------------------------------------------------------
 * CALL BACK HANDLER
 */
class segmenterNodeHandler
{
  public:
    ros::Publisher marker_pub;
    visualizer::BoundingBox bb;
    visualization_msgs::MarkerArray marker_array;
    std::vector<visualization_msgs::Marker> Marker_vector;
    actionlib::SimpleActionClient<robot_vision::SVMclassifierAction> ac;
    robot_vision::SVMclassifierResultConstPtr result;

    segmenterNodeHandler(ros::NodeHandle nh);
    void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    void pushMarker();
};

segmenterNodeHandler::segmenterNodeHandler(ros::NodeHandle nh): bb("world"),ac("SVMAction", true)
{
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker", 1);
}

void
segmenterNodeHandler::cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
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
      pub_segmented.publish(output);
      // Publish Segmented Cloud end

      robot_vision::pointcloudVector pv;
      pcl::PCLPointCloud2 pc;
      sensor_msgs::PointCloud2 outputPC;
      for(size_t i=0;i<finalClusters.size();i++)
      {
        pcl::toPCLPointCloud2(*finalClusters[i],pc);
        pcl_conversions::fromPCL(pc, outputPC);
        pv.pointClouds.push_back(outputPC);
      }
      pub_pcVector.publish(pv);
      ros::Rate r(1);


} //END cloud_cb

void
segmenterNodeHandler::pushMarker()
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
    segmenterNodeHandler cbk(nh);
    ros::Subscriber sub = nh.subscribe ("cloud_pcd", 1, &segmenterNodeHandler::cloud_cb,&cbk);

    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ > >("output", 1);
    pub_segmented = nh.advertise<sensor_msgs::PointCloud2>("segmented", 1);
    pub_pcVector = nh.advertise<robot_vision::pointcloudVector>("pcVector", 1);

    dynamic_reconfigure::Server<robot_vision::robot_vision_paramsConfig> server;
    dynamic_reconfigure::Server<robot_vision::robot_vision_paramsConfig>::CallbackType f;


    f = boost::bind(&reconfigCallback, _1, _2);
    server.setCallback(f);

    ros::spin ();
}
