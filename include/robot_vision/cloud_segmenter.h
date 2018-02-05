#ifndef _CLOUD_SEGMENTER_H_
#define _CLOUD_SEGMENTER_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>
#include <sstream>

class cloud_segmenter{
private:
    pcl::PCDReader reader;
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f;
    std::vector<pcl::PointIndices> cluster_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
  cloud_segmenter(void);
  void load_cloud(std::string);
  void load_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr someCloud);
  void getVoxelFiltered(double leafx, double leafy, double leafz);
  void removePlanes(int maxIter,double distanceThreshold,float percentPC);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getClusters(double tolerance,
                                                               double minClusterSize,
                                                               double maxClusterSize);
};

#endif
