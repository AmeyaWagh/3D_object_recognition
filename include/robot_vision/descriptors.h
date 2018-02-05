#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

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

class processPointCloud{
    private:
        float model_ss_;
    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
        pcl::PointCloud<pcl::VFHSignature308>::Ptr VFH_descriptor;

        pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;

        processPointCloud(void);

        void cloudinput(pcl::PointCloud<pcl::PointXYZ>::Ptr someCloud);

        void getNormals(void);

        void getKeypoints(void);

        void getVFHE(void);

        pcl::PointCloud<pcl::VFHSignature308>::Ptr getDescriptor(void);
    };

#endif
