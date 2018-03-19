#! /usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from common_constants import *
from termprinter import *

def drawMarkers(final_clusters):
    # print(dir(Marker))
    
    markerArray = MarkerArray()
    _id=0
    for cluster in final_clusters:
        _color = COLOR_ARRAY[CLASS_PATH.index(cluster[2])]
        _quat = cluster[4]
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        
        minPT = cluster[0]
        maxPT = cluster[1]
        
        marker.pose.orientation.x = _quat[0]
        marker.pose.orientation.y = _quat[1]
        marker.pose.orientation.z = _quat[2]
        marker.pose.orientation.w = _quat[3]

        marker.pose.position.x = (maxPT[0]+minPT[0])/2.0
        marker.pose.position.y = (maxPT[1]+minPT[1])/2.0 
        marker.pose.position.z = (maxPT[2]+minPT[2])/2.0
        
        marker.scale.x = abs(maxPT[0]-minPT[0])
        marker.scale.y = abs(maxPT[1]-minPT[1])
        marker.scale.z = abs(maxPT[2]-minPT[2])
        # print("scale>>",abs(maxPT[0]-minPT[0]),abs(maxPT[1]-minPT[1]),abs(maxPT[2]-minPT[2]))

        marker.color.r = ((_color & 0x00FF0000) >> 16)/255.0;
        marker.color.g = ((_color & 0x0000FF00) >> 8)/255.0;
        marker.color.b = ((_color & 0x000000FF) >> 0)/255.0;
        marker.color.a = 0.5
        
        marker_text = Marker(type = Marker.TEXT_VIEW_FACING,
            id = _id,
            pose = Pose(Point(maxPT[0], maxPT[1], maxPT[2]), Quaternion(_quat[0], _quat[1], _quat[2], _quat[3])),
            scale = Vector3(0.06, 0.06, 0.06),
            header = marker.header,
            color=ColorRGBA(1.0,0.0,1.0,1.0),
            text="{}".format(cluster[2].strip('_1')))
        _id += 1
        markerArray.markers.append(marker)
        markerArray.markers.append(marker_text)
    
    _id = 0
    for m in markerArray.markers:
        m.id = _id
        _id += 1
    # marker_pub.publish(markerArray)
    rospy.loginfo(OKGREEN+"MARKERS PUBLISHED"+ENDC)
    return markerArray