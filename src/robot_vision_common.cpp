#include <robot_vision/robot_vision_common.h>

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

#include <robot_vision/visualizer.h>

#include <robot_vision/common_constants.h>

namespace robot_vision{
template < typename T >
std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
}

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


void
robot_vision_common::detectObject(
                pcl::PointXYZ &minPT,pcl::PointXYZ &maxPT,
                robot_vision::SVMclassifierResultConstPtr result,
                double threshold, double scale,
                visualizer::BoundingBox &bb,
                                  int _id,
                std::vector<visualization_msgs::Marker> &Marker_vector) // updates Marker vector
{
    int no_of_classes = result->sequence.size();

    for (size_t clf_idx=0; clf_idx<no_of_classes;clf_idx+=2)
    {
        if (result->sequence[clf_idx]==1.0 && result->sequence[clf_idx+1]>threshold){
//            pcl::getMinMax3D(*cloudSegment,minPT,maxPT);
            if(inDimensions(minPT,maxPT,bowlLength,bowlWidth,bowlHeight,scale))
          {
                Marker_vector.push_back(bb.getBoundingBox(minPT.x,minPT.y,minPT.z,
                                                          maxPT.x,maxPT.y,maxPT.z,// x,y,z
                                                          color_array[clf_idx/2],_id));
            }

        }
    }

}
