#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <iostream>
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
#include <pcl/filters/voxel_grid.h>

#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

#include <robot_vision/descriptors.h>
#include <robot_vision/classifier.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

class getPointCloud{
    private :
        struct stat info;
    public :
        std::string PCD_BASE_PATH;
        std::string PCD_CLASS_PATH;
        std::string PCD_FILE_PATH;
        std::string PCD_FILE_EXT;
        std::string FULL_FILE_PATH;
        std::string CLASSIFIER_NAME;
        std::vector<std::string> DataFiles;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::VFHSignature308>::Ptr VFH_descriptor;

        ofstream outputFile;

        processPointCloud pc;
        classifier clf;

        getPointCloud(std::string _PCD_BASE_PATH,
                      std::vector<std::string> _DataFiles,
                      std::string _CLASSIFIER_NAME);

        pcl::PointCloud<pcl::PointXYZ>::Ptr loadCloud(std::string filePath);
        void getTrainingData(cv::Mat& trainingData,cv::Mat& trainingLabels);
        int getdir (std::string dir, std::vector<std::string> &files);

};

/*
 * Constructor
 */
getPointCloud::getPointCloud(std::string _PCD_BASE_PATH,
              std::vector<std::string> _DataFiles,
              std::string _CLASSIFIER_NAME): cloud(new pcl::PointCloud<pcl::PointXYZ>),
                     VFH_descriptor(new pcl::PointCloud<pcl::VFHSignature308>),
                     clf()
{


    PCD_BASE_PATH = _PCD_BASE_PATH;
    CLASSIFIER_NAME = _CLASSIFIER_NAME;
    for(size_t i=0;i<_DataFiles.size();i++){
        DataFiles.push_back(PCD_BASE_PATH +_DataFiles[i]);
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr
getPointCloud::loadCloud(std::string filePath){
    pcl::PointCloud<pcl::PointXYZ>::Ptr someCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, *someCloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    return someCloud;
}


void
getPointCloud::getTrainingData(cv::Mat& trainingData,cv::Mat& trainingLabels){
    float minX,minY,minZ=1000.0;
    float maxX,maxY,maxZ=0.0;

    pcl::VoxelGrid<pcl::PointXYZ> vg;

    cv::Mat label = cv::Mat::zeros(1,1,CV_32FC1);

//         Loop over each Directory
    for(size_t fileIdx=0; fileIdx<DataFiles.size();fileIdx++){
        std::vector<std::string> files = std::vector<std::string>();
        pcl::PointXYZ minPt, maxPt;
        int skip=0;

        getdir(DataFiles[fileIdx]+"/",files);
        ROS_INFO("No of Files: %d",(int)files.size());

        //    cv::Mat trainingData;
        //    cv::Mat trainingLabels;
        // Loop over each file
        for (unsigned int i = 0;i < files.size();i++) {
//                 remove this after testing classifier training
            if(skip > 5){break;}
            skip++;

            if(files[i] == "."  || files[i] == ".."){
                ROS_INFO(" . or .. files ignored");
            }
            else{
                std::cout << " [ processing ] " << files[i];

                cloud = loadCloud(DataFiles[fileIdx]+"/"+files[i]);

                //-- voxelization of the cloud --------//
                vg.setInputCloud (cloud);
                vg.setLeafSize (0.01f, 0.01f, 0.01f);
                vg.filter (*cloud);
                //-- Getting the descriptors ----------//
                pc.cloudinput(cloud);
                VFH_descriptor = pc.getDescriptor();
                cv::Mat _descriptor = cv::Mat::zeros(1,306,CV_32FC1);

                for(size_t i=0;i<306;i++){
                    _descriptor.at<float>(0,i)=(float)VFH_descriptor->points[0].histogram[i];
                }
                //-------------------------------------//
                trainingData.push_back(_descriptor);
                if(DataFiles[fileIdx] == (PCD_BASE_PATH+CLASSIFIER_NAME)){
                    label.at<float>(0,0)=1.0;
                    trainingLabels.push_back(label);
                    std::cout << " [label] 1.0";
                }
                else{
                    label.at<float>(0,0)= -1.0;
                    trainingLabels.push_back(label);
                    std::cout << " [label] -1.0";
                }
                std::cout << std::endl;

            }
        }
        // here
    }
}


int
getPointCloud::getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

int main(int argc, char** argv){
    ros::init(argc,argv,"SVMTrainer");
    std::string pkgPath = ros::package::getPath("robot_vision");
    ROS_INFO("Training SVM");
    std::string PCD_BASE_PATH;
    std::string POSITIVE_CLASS;
    bool parsePCD,trainSVM;
    std::vector<std::string> PCD_CLASS_PATHS;
    ros::param::get("PCD_BASE_PATH",PCD_BASE_PATH);
    ros::param::get("parsePCD",parsePCD);
    ros::param::get("trainSVM",trainSVM);
    ros::param::get("PCD_CLASS_PATHS",PCD_CLASS_PATHS);
    ros::param::get("POSITIVE_CLASS",POSITIVE_CLASS);
    ROS_INFO("Parameters %s %s",PCD_BASE_PATH.c_str(),parsePCD ? "true":"false");

    for(size_t i=0;i<PCD_CLASS_PATHS.size();i++){
        ROS_INFO("class: %s",PCD_CLASS_PATHS[i].c_str());
    }
    ROS_INFO("done");
    std::string trainDataFile = pkgPath+"/bin/trainingData.xml";
    std::string trainLabelsFile = pkgPath+"/bin/trainingLabels.xml";
    std::string classifierPath = pkgPath+"/bin/"+POSITIVE_CLASS;
    ROS_INFO("trainData: %s",trainDataFile.c_str());
    ROS_INFO("trainLabels: %s",trainLabelsFile.c_str());

    classifier clf;

    if(parsePCD){
        cv::Mat trainingData;
        cv::Mat trainingLabels;

        getPointCloud p(PCD_BASE_PATH,PCD_CLASS_PATHS,POSITIVE_CLASS);
        p.getTrainingData(trainingData,trainingLabels);


        ROS_INFO("Training Data dimensions: rows: %d cols: %d",(int)trainingData.rows,(int)trainingData.cols);
        ROS_INFO("Training Labels dimensions: rows: %d cols: %d",(int)trainingLabels.rows,(int)trainingLabels.cols);

        {
            cv::FileStorage file(trainDataFile, cv::FileStorage::WRITE);
            file << "trainingData" << trainingData;
            file.release();
        }
        {

            cv::FileStorage file(trainLabelsFile, cv::FileStorage::WRITE);
            file << "trainingLabels" << trainingLabels;
            file.release();
        }
//        std::cout << "training Data saved" << std::endl;
        ROS_INFO("Training Data saved.");
    }

    if(trainSVM){
        cv::Mat trainingData;
        cv::Mat trainingLabels;
        {
            cv::FileStorage file(trainDataFile, cv::FileStorage::READ);
            file["trainingData"] >> trainingData;
            file.release();
        }
        {
            cv::FileStorage file(trainLabelsFile, cv::FileStorage::READ);
            file["trainingLabels"] >> trainingLabels;
            file.release();
        }

        ROS_INFO("Files Loaded.");
        ROS_INFO("training SVM.");
        clf.trainSVM(trainingData,trainingLabels,classifierPath);
        ROS_INFO("SVM trained.");

    }

    ros::shutdown();
    return 0;
}
