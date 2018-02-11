#!/usr/bin/python
import rospy
import rospkg
from std_msgs.msg import String
# import os
# import numpy as np
# import cv2

# from sklearn.svm import SVC
# from sklearn.externals import joblib
# from sklearn.decomposition import IncrementalPCA

# import matplotlib.pyplot as plt
# from random import shuffle

rospack = rospkg.RosPack()

from robot_vision.datahandler import DataHandler
from robot_vision.classifier import Classifier


if __name__ == '__main__':
    rospy.init_node('trainClassifier', anonymous=True)
    rospy.loginfo("trainClassifier")
    d = DataHandler()
    d.parse_data()
    c = Classifier(d.trainingData,d.trainingLabels,d.testData,d.testLabels)

    # d.train_classifier()
    c.visualize_data()

