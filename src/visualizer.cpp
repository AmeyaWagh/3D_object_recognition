#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <robot_vision/visualizer.h>
#include <std_msgs/ColorRGBA.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

visualizer::BoundingBox::BoundingBox(std::string frame_id){
    shape = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5f;
}

visualization_msgs::Marker 
visualizer::BoundingBox::getBoundingBox(float x,float y,float z,
    float qx,float qy,float qz,float qw){
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;

      marker.pose.orientation.x = qx;
      marker.pose.orientation.y = qy;
      marker.pose.orientation.z = qz;
      marker.pose.orientation.w = qw;
      
      return marker;
}

visualization_msgs::Marker
visualizer::BoundingBox::getBoundingBox(float minX,float minY,float minZ,
                                        float maxX,float maxY,float maxZ,
                                        std::string _color, int _id){


    marker.id = _id;
    marker.pose.position.x = (minX+maxX)/2.0;
    marker.pose.position.y = (minY+maxY)/2.0;
    marker.pose.position.z = (minZ+maxZ)/2.0;

    marker.scale.x = (maxX-minX);
    marker.scale.y = (maxY-minY);
    marker.scale.z = (maxZ-minZ);

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    if (_color == "RED"){
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    }
    else if(_color == "BLUE"){
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    }
    else{
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }

    return marker;
}


