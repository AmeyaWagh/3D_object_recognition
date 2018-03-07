#!/usr/bin/python
import rospy
import rospkg
from std_msgs.msg import String


rospack = rospkg.RosPack()

from robot_vision_helper.datahandler import DataHandler
from robot_vision_helper.classifier import Classifier


if __name__ == '__main__':
    rospy.init_node('trainClassifier', anonymous=True)
    rospy.loginfo("trainClassifier")
    d = DataHandler()
    d.parse_data()
    c = Classifier(d.trainingData,d.trainingLabels,d.testData,d.testLabels)

    # c.train_classifier()
    c.trainFCNet()
    # c.visualize_data()

