#include <robot_vision/robot_vision_common.h>
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

void
robot_vision_common::AssembleCloud(
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &finalClusters,
        sensor_msgs::PointCloud2 &output)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr someCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pc;
    for(size_t i=0;i<finalClusters.size();i++){
        for(size_t j=0;j<finalClusters[i]->points.size();j++){
            someCloud->points.push_back(finalClusters[i]->points.at(j));
        }
    }

    pcl::toPCLPointCloud2(*someCloud,pc);
    pcl_conversions::fromPCL(pc, output);
    output.header.frame_id=std::string("world");
}


bool
robot_vision_common::inDimensions(pcl::PointXYZ minPT,pcl::PointXYZ maxPT,
                  double length,double width,double height,double scale)
{

    if((std::abs(maxPT.y - minPT.y) < scale*width) &&
            (std::abs(maxPT.x - minPT.x) < scale*length) &&
            (std::abs(maxPT.z - minPT.z) < scale*height)){
        return true;
    }
    else{
        return false;
    }
}
