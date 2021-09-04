#ifndef REALSENSE
#define REALSENSE

#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>


// Intel Realsense Headers
#include <pcl/io/real_sense_2_grabber.h>

// PCL Headers
#include "dataType.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>

//OpenCV Headers
#include <opencv2/opencv.hpp>

using namespace std;

class Realsense
{
private:
	//Intrinsics
	/*
	double rgb_fx = 527.360558216633;
	double rgb_fy = 528.3632079059613;
	double rgb_cx = 319.6283688314646;
	double rgb_cy = 263.2500840924527;

	double depth_fx = 585.3081185218954;
	double depth_fy = 587.28826280342;
	double depth_cx = 317.8356267919822;
	double depth_cy = 247.1889948214958;
	*/

	
	//Grabber
	std::shared_ptr<pcl::RealSense2Grabber> grabber;
	//Callbacks
	std::function<void(const std::shared_ptr<const PointCloudType >& cloud)> f_cloud;
	std::function<void(const std::shared_ptr<rs2::video_frame>& image)> f_image;

	boost::signals2::connection connection_cloud;
	boost::signals2::connection connection_image;
	void initIntrinsics();
public:
	std::deque<cv::Mat> images_q;
	Realsense() {}
	void registerCallbackFunction(std::function<void(const std::shared_ptr<const PointCloudType>& cloud)> lambda_f_cloud, std::function<void(const std::shared_ptr<rs2::video_frame>& image)> lambda_f_image);
	bool run();
	void stop();
};

#endif //REALSENSE