#! /usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import String
from robot_vision.msg import pointcloudVector
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np
import cv2
import os
from robot_vision_helper.GASD import GASD
from robot_vision_helper.CNNModel import CNNModel
from robot_vision_helper.DataHandler import DataHandler
import tensorflow as tf
from robot_vision_helper.termprinter import *
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path('robot_vision')
print(OKBLUE+PACKAGE_PATH+ENDC)

cnn = CNNModel(ip_shape=(1, 40, 40, 40))
cnn.load_model(base_path = os.path.join(PACKAGE_PATH,"bin/3DCNN_model"))
BOX_SIZE = (0.3,0.3,0.3)
PRED_THRESHOLD = 0.99

RED = 0xFF0000
GREEN = 0x00FF00
BLUE = 0x0000FF
YELLOW = 0xFFFF00
CYAN = 0x00FFFF
MAGENTA = 0xFF00FF

COLOR_ARRAY = [RED,GREEN,BLUE,YELLOW,CYAN,MAGENTA]

CLASS_PATH = ["apple_1","banana_1","bowl_1","calculator_1","coffee_mug_1"]


# Call back runs on a different thread, thus we have to make graph global
# ref : https://www.tensorflow.org/api_docs/python/tf/get_default_graph
graph = tf.get_default_graph()

def is_in_dimension(cloud_array):
    max_dim = np.amax(cloud_array,axis=0)
    min_dim = np.amin(cloud_array,axis=0)
    # print("size:")
    return all((max_dim-min_dim)<BOX_SIZE),(max_dim,min_dim)

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
    marker_pub.publish(markerArray)
    rospy.loginfo(OKGREEN+"MARKERS PUBLISHED"+ENDC)    



class NodeHandler:
    def __init__(self):
        self.g = GASD()
        self.CLASS_PATH = ["apple_1","banana_1","bowl_1","calculator_1","coffee_mug_1"]
        self.d = DataHandler()

    def pc_to_numpy(self,cloud):
        cloud_array = []
        for p in pc2.read_points(cloud, field_names = ("x", "y", "z"), skip_nans=True):
            cloud_array.append(p)
        cloud_array = np.array(cloud_array)
        # print cloud_array
        return cloud_array
    
    def callback(self,data):
        global graph
        print("\n")
        rospy.loginfo(rospy.get_caller_id() + "received %d segments", len(data.pointClouds))
        final_clusters = []
        with graph.as_default():
            for cloud in data.pointClouds:
                cloud_array = self.pc_to_numpy(cloud)
                dim_check,minmaxpt = is_in_dimension(cloud_array)
                if dim_check:
                    volume_data,quaternion = self.g.get_volumetric_data(cloud_array)
                    label,proba = cnn.predict(np.array([[volume_data]]),self.CLASS_PATH)
                    if proba >= PRED_THRESHOLD:
                        rospy.loginfo(" Label {} - Probability {}".format(label,proba))
                        final_clusters.append((minmaxpt[0],minmaxpt[1],label,proba,quaternion))
        drawMarkers(final_clusters)        
    
    def listener(self):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("pcVector", pointcloudVector, self.callback)
        global marker_pub
        marker_pub = rospy.Publisher('visualization_marker', MarkerArray)

        rospy.loginfo("Starting classifierNode")
        rospy.spin()

if __name__ == '__main__':
    nh = NodeHandler()
    nh.listener()