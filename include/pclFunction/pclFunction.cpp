#include "pclFunction.h"

void CustomVisualizer::init() {
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(0.05);
	viewer->initCameraParameters();
}

/**
* @brief The PassThrough filter is used to identify and/or eliminate points
within a specific range of X, Y and Z values.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter - cloud after the application of the filter
* @return void
*/

void passthrough(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, std::vector<float> limits, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	pcl::PassThrough<pcl::PointXYZ> pt;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pt.setInputCloud(cloud_in);
	pt.setFilterFieldName("z");
	//pt.setFilterLimits(0.0, 0.85);//0.81
	pt.setFilterLimits(limits[0], limits[1]);//0.81
	pt.filter(*cloud_ptz_ptr);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptx_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pt.setInputCloud(cloud_ptz_ptr);
	pt.setFilterFieldName("x");
	//pt.setFilterLimits(-0.3, 0.3);//0.37
	pt.setFilterLimits(limits[2], limits[3]);//0.81
	pt.filter(*cloud_ptx_ptr);

	pt.setInputCloud(cloud_ptx_ptr);
	pt.setFilterFieldName("y");
	//pt.setFilterLimits(-0.25, 0.25);//0.2
	pt.setFilterLimits(limits[4], limits[5]);//0.81
	pt.filter(*cloud_out);
}

void passthrough(const PointCloudType::ConstPtr& cloud_in, std::vector<float> limits, PointCloudType::Ptr& cloud_out)
{
	pcl::PassThrough<PointType> pt;
	
	pcl::PointCloud<PointType>::Ptr cloud_ptz_ptr(new pcl::PointCloud<PointType>);
	pt.setInputCloud(cloud_in);
	pt.setFilterFieldName("z");
	//pt.setFilterLimits(0.0, 0.85);//0.81
	pt.setFilterLimits(limits[0], limits[1]);//0.81
	pt.filter(*cloud_ptz_ptr);

	pcl::PointCloud<PointType>::Ptr cloud_ptx_ptr(new pcl::PointCloud<PointType>);
	pt.setInputCloud(cloud_ptz_ptr);
	pt.setFilterFieldName("x");
	//pt.setFilterLimits(-0.3, 0.3);//0.37
	pt.setFilterLimits(limits[2], limits[3]);//0.81
	pt.filter(*cloud_ptx_ptr);

	pt.setInputCloud(cloud_ptx_ptr);
	pt.setFilterFieldName("y");
	//pt.setFilterLimits(-0.25, 0.25);//0.2
	pt.setFilterLimits(limits[4], limits[5]);//0.81
	pt.filter(*cloud_out);
}

/*
* @brief The Statistical Outliner Remove filter is used to remove noisy measurements, e.g. outliers, from a point cloud dataset
  using statistical analysis techniques.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void statisticalOutlinerRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int numNeighbors, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setMeanK(numNeighbors);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_out);
}

/*
* @brief The VoxelGrid filter is used to simplify the cloud, by wrapping the point cloud
  with a three-dimensional grid and reducing the number of points to the center points within each bloc of the grid.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void voxelgrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float size_leaf, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	// cout << "So diem truoc khi loc voxel " << cloud_in->points.size() << endl;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud_in);
	vg.setLeafSize(size_leaf, size_leaf, size_leaf); // 3mm
	vg.setDownsampleAllData(true);
	vg.filter(*cloud_out);
	//  cout << "So diem sau khi loc voxel " << cloud_out->points.size() << endl;
}

/*
* @brief The SACSegmentation and the ExtractIndices filters are used to identify and
remove the table from the point cloud leaving only the objects.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter - cloud after the application of the filter
* @return void
*/
void sacsegmentation_extindices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,double dist_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	// Identify the table
	pcl::PointIndices::Ptr sacs_inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr sacs_coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> sacs;
	sacs.setOptimizeCoefficients(true);
	sacs.setModelType(pcl::SACMODEL_PLANE);
	sacs.setMethodType(pcl::SAC_RANSAC);
	sacs.setMaxIterations(900);//900
	sacs.setDistanceThreshold(dist_threshold);//16mm
	sacs.setInputCloud(cloud_in);
	sacs.segment(*sacs_inliers, *sacs_coefficients);
	// Remove the table
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setInputCloud(cloud_in);
	ei.setIndices(sacs_inliers);
	ei.setNegative(true);
	ei.filter(*cloud_out);
}

void remove_background(const pcl::PointCloud<PointType>::Ptr& cloud_in, double dist_threshold, pcl::PointCloud<PointType>::Ptr& cloud_out)
{
	// Identify the table
	pcl::PointIndices::Ptr sacs_inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr sacs_coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<PointType> sacs;
	sacs.setOptimizeCoefficients(true);
	sacs.setModelType(pcl::SACMODEL_PLANE);
	sacs.setMethodType(pcl::SAC_RANSAC);
	sacs.setMaxIterations(900);//900
	sacs.setDistanceThreshold(dist_threshold);//16mm
	sacs.setInputCloud(cloud_in);
	sacs.segment(*sacs_inliers, *sacs_coefficients);
	// Remove the table
	pcl::ExtractIndices<PointType> ei;
	ei.setInputCloud(cloud_in);
	ei.setIndices(sacs_inliers);
	ei.setNegative(true);
	ei.filter(*cloud_out);
}

double computeCloudDiameter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.005f, 0.005f, 0.005f);
	vg.setDownsampleAllData(false);
	vg.filter(*cloud_downsampled);

	double diameter_sqr = 0;
	for (size_t i = 0; i < cloud_downsampled->points.size(); i += 10)
	{
		for (size_t j = 0; j < cloud_downsampled->points.size(); j += 10)
		{
			if (i == j)
				continue;
			double distance_sqr = (cloud_downsampled->points[i].x - cloud_downsampled->points[j].x)*(cloud_downsampled->points[i].x - cloud_downsampled->points[j].x)
				+ (cloud_downsampled->points[i].y - cloud_downsampled->points[j].y)*(cloud_downsampled->points[i].y - cloud_downsampled->points[j].y)
				+ (cloud_downsampled->points[i].z - cloud_downsampled->points[j].z)*(cloud_downsampled->points[i].z - cloud_downsampled->points[j].z);
			if (distance_sqr > diameter_sqr)
			{
				diameter_sqr = distance_sqr;
			}
		}
	}
	return sqrt(diameter_sqr);
}
