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
from robot_vision_helper.common_constants import *
from robot_vision_helper.bounding_box import *

rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path('robot_vision')
print(OKBLUE+PACKAGE_PATH+ENDC)


""" 3D-CNN is initailized to have grid size 40x40x40 """
cnn = CNNModel(ip_shape=(1, 40, 40, 40))
cnn.load_model(base_path=os.path.join(PACKAGE_PATH, "bin/3DCNN_model"))


# Call back runs on a different thread, thus we have to make graph global
# ref : https://www.tensorflow.org/api_docs/python/tf/get_default_graph
graph = tf.get_default_graph()


def is_in_dimension(cloud_array):
    """ check if the cloud segment is of required dimension i.e mxmxm"""
    max_dim = np.amax(cloud_array, axis=0)
    min_dim = np.amin(cloud_array, axis=0)
    return all((max_dim-min_dim) < BOX_SIZE), (max_dim, min_dim)


class NodeHandler:
    """ Handler for classification node"""

    def __init__(self):
        self.g = GASD()
        self.CLASS_PATH = ["apple_1", "banana_1",
                           "bowl_1", "calculator_1", "coffee_mug_1"]
        self.d = DataHandler()

    def pc_to_numpy(self, cloud):
        """ convert the PointCloud2 object to a numpy array """
        cloud_array = []
        for p in pc2.read_points(cloud, field_names=("x", "y", "z"),
                                 skip_nans=True):
            cloud_array.append(p)
        cloud_array = np.array(cloud_array)
        return cloud_array

    def callback(self, data):
        """ Listern to topic /pcVector which is an array of 
            segmented pointclouds of type PointCloud2. 
            Classify each PointCloud and publish bounding boxes 
            to topic /visualization_marker """

        global graph
        print("\n")
        rospy.loginfo(rospy.get_caller_id() +
                      "received %d segments", len(data.pointClouds))
        final_clusters = []
        with graph.as_default():
            # iterate over every segment of PointClouds
            for cloud in data.pointClouds:
                cloud_array = self.pc_to_numpy(cloud)
                dim_check, minmaxpt = is_in_dimension(cloud_array)
                
                if dim_check:
                    volume_data, quaternion = self.g.get_volumetric_data(
                        cloud_array)
                    label, proba = cnn.predict(
                        np.array([[volume_data]]), self.CLASS_PATH)
                    
                
                    # check if prediction probability is above required threshold
                    # and suppress unnecessary labels
                    if (proba >= PRED_THRESHOLD) and (label in ["coffee_mug_1",
                                                                "bowl_1"]):
                        rospy.loginfo(
                            " Label {} - Probability {}".format(label, proba))
                        final_clusters.append((minmaxpt[0],
                                               minmaxpt[1], label,
                                               proba, quaternion))
        marker_pub.publish(drawMarkers(final_clusters))

    def listener(self):
        """ Initialize topics and start the node """
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("pcVector", pointcloudVector, self.callback)
        global marker_pub
        marker_pub = rospy.Publisher('visualization_marker', MarkerArray)

        rospy.loginfo("Starting classifierNode")
        rospy.spin()


if __name__ == '__main__':
    nh = NodeHandler()
    nh.listener()
