#ifndef _CLASSIFIER_H_
#define _CLASSIFIER_H_


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/common/common.h>

#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <stdlib.h>
#include <time.h>



class classifier{
private:
    CvSVM SVMclf;
public:
    classifier(void);
    classifier(std::string filename);
    void trainSVM(cv::Mat _trainingData, cv::Mat _trainingLabels);
    void trainSVM(cv::Mat _trainingData, cv::Mat _trainingLabels,std::string fileName);

    float getConfidence(float distance);
    void validateSVM(cv::Mat _testData, cv::Mat _testLabels);
    std::vector<float> predict(cv::Mat query);
    cv::Mat vector2Mat(pcl::PointCloud<pcl::VFHSignature308>::Ptr inputDescriptor);

};

#endif
