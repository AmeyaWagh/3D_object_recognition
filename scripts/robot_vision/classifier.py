#!/usr/bin/python
import rospy
import rospkg
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn.decomposition import IncrementalPCA

import numpy as np
import os

import matplotlib.pyplot as plt

from keras.models import Sequential
from keras.layers import Dense

from robot_vision import termcolors


rospack = rospkg.RosPack()

class Classifier:
    def __init__(self, trainingData, trainingLabels, testData, testLabels):
        self.trainingData = trainingData
        self.trainingLabels = trainingLabels
        self.testData = testData
        self.testLabels = testLabels
        self.PKG_PATH = rospack.get_path('robot_vision')
        self.MODEL_PATH = '.'

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

    def trainFCNet(self,epochs=10,verbose=True):
        model = Sequential()
        model.add(Dense(60, input_dim=306, kernel_initializer='normal', activation='relu'))  
        model.add(Dense(1, kernel_initializer='normal', activation='sigmoid'))
        model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
        model.fit(self.trainingData,self.trainingLabels, batch_size=32, epochs=epochs, verbose= 1 if verbose else 0)
        score = model.evaluate(self.testData, self.testLabels, verbose=1)
        rospy.loginfo("{}[ LOSS] {}   [ ACCURACY] {}{}".format(termcolors.OKGREEN,score[0],score[1],termcolors.ENDC))
        
        with open(os.path.join(self.PKG_PATH,"bin",self.MODEL_PATH,"model.json"), "w") as json_file:
            json_file.write(model.to_json())
        model.save_weights(os.path.join(self.PKG_PATH,"bin",self.MODEL_PATH,"model.h5"))
        rospy.loginfo("Saved model to disk") 

    def trainFCNetCaffe(self):
        pass 
 

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
