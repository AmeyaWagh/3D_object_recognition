#!/usr/bin/python
import rospy
import rospkg
from std_msgs.msg import String
import os
import numpy as np
import cv2

from random import shuffle

rospack = rospkg.RosPack()



class DataHandler:
    def __init__(self, ratio=0.8):
        self.ratio = ratio
        self.pkgPath = rospack.get_path('robot_vision')
        rospy.loginfo("pkgPath {}".format(self.pkgPath))
        self.BASE_PATH = "/home/ameya/pcl_ws/src/robot_vision/bin/Dataset"
        self.DATAFILES = os.listdir(self.BASE_PATH)
        self.DATAFILES_PATH = [os.path.join(
            self.BASE_PATH, filePath) for filePath in self.DATAFILES]
        rospy.loginfo(self.DATAFILES)
        self.trainingData = np.array([])
        self.trainingLabels = np.array([])
        self.POSITIVE_CLASS = "bowl_1"

    def parse_data(self):
        for i in range(len(self.DATAFILES_PATH)):
            fs = cv2.FileStorage(self.DATAFILES_PATH[i], cv2.FILE_STORAGE_READ)
            CLASS_NAME = self.DATAFILES[i].strip(".xml")
            data = fs.getNode(CLASS_NAME).mat()
            print(np.shape(data))
            if self.trainingData.size == 0:
                self.trainingData = data
                if CLASS_NAME == self.POSITIVE_CLASS:
                    self.trainingLabels = np.ones(
                        (np.shape(data)[0], 1), dtype=float)
                else:
                    self.trainingLabels = np.zeros(
                        (np.shape(data)[0], 1), dtype=float)
            else:
                self.trainingData = np.vstack((self.trainingData, data))
                if CLASS_NAME == self.POSITIVE_CLASS:
                    self.trainingLabels = np.vstack(
                        (self.trainingLabels, np.ones(
                            (np.shape(data)[0], 1), dtype=float)))
                else:
                    self.trainingLabels = np.vstack(
                        (self.trainingLabels, np.zeros(
                            (np.shape(data)[0], 1), dtype=float)))

        # print(self.trainingData)
        # print(self.trainingLabels)
        rospy.loginfo("TotalData {}".format(np.shape(self.trainingData)))
        rospy.loginfo("TotalLabels {}".format(
            np.shape(self.trainingLabels)))
        dataZip = [z for z in zip(self.trainingData, self.trainingLabels)]
        shuffle(dataZip)
        shuffle(dataZip)
        shuffle(dataZip)
        self._bufferData = np.array([data[0] for data in dataZip])
        self._bufferLabels = np.array([data[1] for data in dataZip])
        (r, c) = np.shape(self._bufferData)
        self.trainingData = self._bufferData[:int(r * self.ratio), :]
        self.trainingLabels = self._bufferLabels[:int(
            len(self._bufferLabels) * self.ratio)]

        self.testData = self._bufferData[int(r * self.ratio):, :]
        self.testLabels = self._bufferLabels[int(
            len(self._bufferLabels) * self.ratio):]

        rospy.loginfo("trainingData-> {}".format(np.shape(np.array(self.trainingData))))
        rospy.loginfo("trainingLabels-> {}".format(np.shape(np.array(self.trainingLabels))))

        rospy.loginfo("testData-> {}".format(np.shape(np.array(self.testData))))
        rospy.loginfo("testLabels-> {}".format(np.shape(np.array(self.testLabels))))

        return self.trainingData,self.trainingLabels,self.testData,self.testLabels



