#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import String
import os
import numpy as np
import cv2
from sklearn.svm import SVC
from sklearn.externals import joblib

rospack = rospkg.RosPack()

class DataHandler:
	def __init__(self):
		self.pkgPath = rospack.get_path('robot_vision')
		rospy.loginfo("pkgPath {}".format(self.pkgPath))
		self.BASE_PATH ="/home/ameya/pcl_ws/src/robot_vision/bin/Dataset"
		self.DATAFILES = os.listdir(self.BASE_PATH)
		self.DATAFILES_PATH = [os.path.join(self.BASE_PATH,filePath) for filePath in self.DATAFILES ]
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
					self.trainingLabels = np.ones((np.shape(data)[0],1), dtype=float)
				else:
					self.trainingLabels = np.zeros((np.shape(data)[0],1), dtype=float)
			else:
				self.trainingData = np.vstack((self.trainingData,data))	
				if CLASS_NAME == self.POSITIVE_CLASS:
					self.trainingLabels = np.vstack((self.trainingLabels,np.ones((np.shape(data)[0],1), dtype=float)))
				else:
					self.trainingLabels = np.vstack((self.trainingLabels,np.zeros((np.shape(data)[0],1), dtype=float)))

		print(self.trainingData)	
		print(self.trainingLabels)
		rospy.loginfo("trainingData {}".format(np.shape(self.trainingData)))
		rospy.loginfo("trainingLabels {}".format(np.shape(self.trainingLabels)))
	
	def trainClassifier(self):
		self.clf = SVC(probability=True,verbose=True)
		self.clf.fit(self.trainingData,self.trainingLabels.reshape(np.shape(self.trainingLabels)[0],))
		joblib.dump(self.clf, os.path.join(self.pkgPath,'bin','pcl_svm_{}.pkl'.format(self.POSITIVE_CLASS)))
		rospy.loginfo("training complete")
		score = self.clf.score(self.trainingData, self.trainingLabels)
		print("score",score)		



if __name__ == '__main__':
	rospy.init_node('trainClassifier', anonymous=True)
	rospy.loginfo("trainClassifier")
	d = DataHandler()
	d.ParseData()
	d.trainClassifier()
