//#ifndef PCLFUNCTION
//#define PCLFUNCTION

#pragma once

#include "dataType.h"

#include <math.h>
#include <type_traits> 

// filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
// segment
#include <pcl/segmentation/sac_segmentation.h>

// visualize
#include <pcl/visualization/pcl_visualizer.h>
/**
* @brief The PCL Visualizer is used to visualize multiple points clearly.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
class CustomVisualizer
{
public:
	bool ready = true;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	CustomVisualizer() :viewer(pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"))) {}
	void init();
};

/**
* @brief The PassThrough filter is used to identify and/or eliminate points
within a specific range of X, Y and Z values.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void passthrough(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, std::vector<float> limits, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
void passthrough(const PointCloudType::ConstPtr& cloud_in, std::vector<float> limits, PointCloudType::Ptr& cloud_out);

/*
* @brief The Statistical Outliner Remove filter is used to remove noisy measurements, e.g. outliers, from a point cloud dataset 
  using statistical analysis techniques.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void statisticalOutlinerRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int numNeighbors, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
/*
* @brief The VoxelGrid filter is used to simplify the cloud, by wrapping the point cloud
  with a three-dimensional grid and reducing the number of points to the center points within each bloc of the grid.
* @param cloud_in - input cloud
* @param cloud_vg_ptr - cloud after the application of the filter
* @return void
*/
void voxelgrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float size_leaf, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

/*
* @brief The SACSegmentation and the ExtractIndices filters are used to identify and
remove the table from the point cloud leaving only the objects.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter - cloud after the application of the filter
* @return void
*/
void sacsegmentation_extindices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, double dist_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

void remove_background(const pcl::PointCloud<PointType>::Ptr& cloud_in, double dist_threshold, pcl::PointCloud<PointType>::Ptr& cloud_out);

double computeCloudDiameter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

//#endif
