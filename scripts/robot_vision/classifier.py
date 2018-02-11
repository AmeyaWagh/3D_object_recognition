#!/usr/bin/python
import rospy
import rospkg
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn.decomposition import IncrementalPCA

import numpy as np

import matplotlib.pyplot as plt


class Classifier:
    def __init__(self, trainingData, trainingLabels, testData, testLabels):
        self.trainingData = trainingData
        self.trainingLabels = trainingLabels
        self.testData = testData
        self.testLabels = testLabels

    def train_classifier(self):
        self.clf = SVC(probability=True, verbose=True)
        self.clf.fit(self.trainingData, self.trainingLabels.reshape(
            np.shape(self.trainingLabels)[0],))

        joblib.dump(self.clf, os.path.join(
            self.pkgPath, 'bin', 'pcl_svm_{}.pkl'.format(
                    self.POSITIVE_CLASS)))

        rospy.loginfo("training complete")
        score = self.clf.score(self.trainingData, self.trainingLabels)
        print("score", score)

    def visualize_data(self):
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

        # rospy.loginfo(np.shape(X1))
        # rospy.loginfo(np.shape(Y1))
        # rospy.loginfo(np.shape(X2))
        # rospy.loginfo(np.shape(Y2))
        rospy.loginfo("PLOTTING GRAPH")
        self.ax.plot(X1, Y1, 'r.', X2, Y2, 'g.')
        plt.show()
        