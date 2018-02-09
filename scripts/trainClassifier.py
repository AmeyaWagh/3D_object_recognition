#!/usr/bin/python
import rospy
import rospkg
from std_msgs.msg import String
import os
import numpy as np
import cv2

from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn.decomposition import IncrementalPCA

# import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

from random import shuffle

rospack = rospkg.RosPack()


class DataHandler:
    def __init__(self):
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

    def ParseData(self):
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

        print(self.trainingData)
        print(self.trainingLabels)
        rospy.loginfo("trainingData {}".format(np.shape(self.trainingData)))
        rospy.loginfo("trainingLabels {}".format(
            np.shape(self.trainingLabels)))
        dataZip = [z for z in zip(self.trainingData , self.trainingLabels)]
        shuffle(dataZip)
        shuffle(dataZip)
        shuffle(dataZip)
        print("->",np.shape(np.array([ data[0] for data in dataZip])))
        print("->",np.shape(np.array([ data[1] for data in dataZip])))
        self.trainingData = np.array([ data[0] for data in dataZip])
        self.trainingLabels = np.array([ data[1] for data in dataZip])

    def trainClassifier(self):
        self.clf = SVC(probability=True, verbose=True)
        self.clf.fit(self.trainingData, self.trainingLabels.reshape(
            np.shape(self.trainingLabels)[0],))

        joblib.dump(self.clf, os.path.join(
            self.pkgPath, 'bin', 'pcl_svm_{}.pkl'.format(
                    self.POSITIVE_CLASS)))

        rospy.loginfo("training complete")
        score = self.clf.score(self.trainingData, self.trainingLabels)
        print("score", score)

    def visualizeData(self):
        ipca = IncrementalPCA(n_components=2, batch_size=3)
        ipca.fit(self.trainingData)

        self.fig = plt.figure()
        # self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax = self.fig.add_subplot(111)
        projData = ipca.transform(self.trainingData)
        print(np.shape(projData))
        X1 = []
        X2 = []
        Y1 = []
        Y2 = []
        for idx in range(len(projData)):
            if self.trainingLabels[idx]:
                X1.append(projData[idx][0])
                Y1.append(projData[idx][1])
            else:
                X2.append(projData[idx][0])
                Y2.append(projData[idx][1])
        # X = np.array([ data[0] for data in projData])
        # Y = np.array([ data[1] for data in projData])
        X1 = np.array(X1)
        X2 = np.array(X2)
        Y1 = np.array(Y1)
        Y2 = np.array(Y2)
        
        print(np.shape(X1))
        print(np.shape(Y1))
        print(np.shape(X2))
        print(np.shape(Y2))

        self.ax.plot(X1,Y1,'r.',X2,Y2,'g.')
        plt.show()    


if __name__ == '__main__':
    rospy.init_node('trainClassifier', anonymous=True)
    rospy.loginfo("trainClassifier")
    d = DataHandler()
    d.ParseData()
    d.trainClassifier()
    d.visualizeData()
