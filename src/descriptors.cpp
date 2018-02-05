#include <robot_vision/descriptors.h>

#include <iostream>
#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/shot_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/common/common.h>


processPointCloud::processPointCloud(void):cloud(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_normals(new pcl::PointCloud<pcl::Normal>),
    VFH_descriptor(new pcl::PointCloud<pcl::VFHSignature308>){
    float model_ss_ (0.001f);
}

void processPointCloud::cloudinput(pcl::PointCloud<pcl::PointXYZ>::Ptr someCloud){
    cloud = someCloud;
}

void processPointCloud::getNormals(void){
    norm_est.setKSearch (10);
    norm_est.setInputCloud (cloud);
    norm_est.compute (*cloud_normals);
}

void processPointCloud::getKeypoints(void){
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.filter(*cloud_keypoints);
}

void processPointCloud::getVFHE(void){
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    vfh.setInputCloud(cloud);
    vfh.setInputNormals(cloud_normals);
    vfh.setSearchMethod(kdtree);
    vfh.setNormalizeBins(true);
    vfh.setNormalizeDistance(false);
    vfh.compute(*VFH_descriptor);
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr
processPointCloud::getDescriptor(void){
    getNormals();
    getKeypoints();
    getVFHE();
    return VFH_descriptor;
}
