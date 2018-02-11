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

namespace robot_vision_common{

    void AssembleCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &finalClusters,
                       sensor_msgs::PointCloud2 &output);

    bool inDimensions(pcl::PointXYZ minPT,pcl::PointXYZ maxPT,
                      double length,double width,double height,double scale);

    bool detectObject(pcl::PointXYZ minPT,pcl::PointXYZ maxPT,
            std::vector<visualization_msgs::Marker> &Marker_vector); // updates Marker vector
}

#endif
