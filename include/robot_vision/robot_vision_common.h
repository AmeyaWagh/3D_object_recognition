#ifndef _ROBOT_VISION_COMMON_H_
#define _ROBOT_VISION_COMMON_H_


#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <robot_vision/SVMclassifierAction.h>

//#ifndef _COMMON_CONSTANTS_H_
#include <robot_vision/common_constants.h>
//#endif
#include <robot_vision/visualizer.h>

namespace robot_vision_common{

    template < typename T > std::string to_string( const T& n );

    void AssembleCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &finalClusters,
                       sensor_msgs::PointCloud2 &output);

    bool inDimensions(pcl::PointXYZ minPT,pcl::PointXYZ maxPT,
                      double length,double width,double height,double scale);

    void detectObject(
                      pcl::PointXYZ &minPT,pcl::PointXYZ &maxPT,
                      robot_vision::SVMclassifierResultConstPtr result,
                      double threshold, double scale,
                      visualizer::BoundingBox &bb,
                      int _id,
                      std::vector<visualization_msgs::Marker> &Marker_vector); // updates Marker vector
}

#endif
