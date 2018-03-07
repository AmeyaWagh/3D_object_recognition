import numpy as np
import os
import pcl
from robot_vision_helper.DisplayCloud import DisplayCloudVispy
import rospy
from termprinter import *

class DataHandler:
    def __init__(self):
        self.BASE_PATH = "/home/ameya/Downloads/Dataset_RGBD"
        self.CLASS_PATH = "apple_1"
        self.FILE_NAME = "apple_1_4_9.pcd"
    
    def load_files(self,BASE_PATH="/home/ameya/Downloads/Dataset_RGBD",CLASS_PATH = ["apple_1"],max_num=None):
        self.BASE_PATH = BASE_PATH
        clouds = []
        labels = []
        print(BOLD+"[ INFO] Reading files from {}".format(self.BASE_PATH)+ENDC)
        for _class_path in CLASS_PATH:
            print(BOLD+"[ classname] {}".format(_class_path)+ENDC)
            idx=0
            for _file_name in os.listdir(os.path.join(self.BASE_PATH,self.CLASS_PATH)):
                if (max_num is not None) and (idx >= max_num):
                    break
                print("[ filename] {}".format(_file_name))
                rospy.loginfo(_file_name)
                _array,_data,_shape = self.load_file(BASE_PATH=self.BASE_PATH,CLASS_PATH=_class_path,file_name=_file_name) 
                # _data = np.resize(_data, (80,80,80))
                _data = np.resize(_data, (40,40,40))
                _data = np.array([_data])
                # print(_shape)
                # print(_data.shape)
                clouds.append(_data)
                labels.append(_class_path)
                idx+=1
        return np.array(clouds),np.array(labels)

    def load_file(self,BASE_PATH="/home/ameya/Downloads/Dataset_RGBD",CLASS_PATH="apple_1",file_name="apple_1_4_9.pcd"):
        self.p = pcl.load(os.path.join(self.BASE_PATH,
                            self.CLASS_PATH,
                            file_name))
        self.p_array = self.p.to_array()*1000
        self.p_array = self.p_array.astype(int)
        self.make_positive()
        _data,_shape = self.get_volume()
        return self.p_array,_data,_shape

    def load_raw_file(self,BASE_PATH="/home/ameya/Downloads/Dataset_RGBD",CLASS_PATH="apple_1",file_name="apple_1_4_9.pcd"):
        # print "BASE_PATH",BASE_PATH
        # print "CLASS_PATH",CLASS_PATH
        # print "file_name",file_name
        # print "fullpath",os.path.join(BASE_PATH,CLASS_PATH,file_name)
        self.p = pcl.load(os.path.join(BASE_PATH,CLASS_PATH,file_name))
        # self.p_array = self.p.to_array()*1000
        # self.p_array = self.p_array.astype(int)
        # self.make_positive()
        # _data,_shape = self.get_volume()
        return self.p.to_array()

    def make_positive(self):
        min_val = np.amin(self.p_array,axis=0)
        self.p_array = self.p_array-min_val 

    def get_volume(self):
        max_val = np.amax(self.p_array,axis=0)
        # print "max",tuple(max_val+1)
        self.volume_data = np.zeros(tuple(max_val+1))
        # print np.shape(self.volume_data)
        for row in self.p_array:
            # print(row)
            self.volume_data[row[0]][row[1]][row[2]] = 1.0
        # print self.volume_data
        return self.volume_data,self.volume_data.shape  