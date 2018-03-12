#! /usr/bin/env python
import rospkg
import numpy as np
import os
import pcl
import cv2
from robot_vision_helper.DataHandler import DataHandler
from robot_vision_helper.GASD import GASD
from robot_vision_helper.CNNModel import *
import time  
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path('robot_vision')
print(OKBLUE+PACKAGE_PATH+ENDC)


class DisplayCloud():
    """ A simple visualizer using Matplotlib """
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def drawCloud(self,pointCloud,pts='.'):
        x = []
        y = []
        z = []
        for pt in pointCloud:
            x.append(float(pt[0]))
            y.append(float(pt[1]))
            z.append(float(pt[2]))

        self.ax.plot(x, y, z, pts)

    def showCloud(self):
        plt.show()

if __name__ == '__main__':
    d = DataHandler()
    g = GASD()
    BASE_PATH = "/home/ameya/Downloads/Dataset_RGBD"
    CLASS_PATH = ["apple_1","banana_1","bowl_1","calculator_1","coffee_mug_1"]
    train_flag = True

    clf = CNNModel(ip_shape=(1, 40, 40, 40))
    if train_flag:
        dataArray = []
        TestdataArray = []
        Labels = []
        TestLabels = []
        dataPercent = 0.8
        for _CLASS in CLASS_PATH:
            files = os.listdir(os.path.join(BASE_PATH,_CLASS))
            arrayLen = len(files)
            i=0
            for _file in files[0:500]:
                print(_file[0:500])
                data = d.load_raw_file(BASE_PATH=BASE_PATH,CLASS_PATH=_CLASS,file_name=_file)
                volume_data,_quat = g.get_volumetric_data(data)
                if i < dataPercent*arrayLen: 
                    dataArray.append(np.array([volume_data]))
                    Labels.append(_CLASS)
                else:
                    TestdataArray.append(np.array([volume_data]))
                    TestLabels.append(_CLASS)

        dataArray = np.array(dataArray)
        TestdataArray = np.array(dataArray)
        Labels = np.array(Labels)
        TestLabels = np.array(Labels)
        print("dataArray",dataArray.shape)
        print("Labels",Labels.shape)

        clf.train(dataArray,Labels)
        clf.test(TestdataArray, TestLabels)
        clf.save_model(base_path=os.path.join(PACKAGE_PATH,"bin/3DCNN_model"))
    
    clf.load_model(base_path=os.path.join(PACKAGE_PATH,"bin/3DCNN_model"))
    for i in range(2):
        query_class = CLASS_PATH[np.random.randint(len(CLASS_PATH))]
        query_file = os.listdir(os.path.join(BASE_PATH,query_class))[-1*np.random.randint(20)]
        data = d.load_raw_file(BASE_PATH = "/home/ameya/Downloads/Dataset_RGBD",CLASS_PATH=query_class,file_name=query_file)
        centroid,axes,new_data = g.compute_gasd(data)
        volume_data,_quat = g.get_volumetric_data(data)
        print("prediction:",clf.predict(np.array([[volume_data]]),CLASS_PATH)," actual:",query_class," file:",query_file)
        disp = DisplayCloud()
        disp.drawCloud(new_data)
        disp.showCloud()

    exit()

