#include "realsense.h"

void Realsense::initIntrinsics()
{
	grabber = std::make_shared<pcl::RealSense2Grabber>();
	//interface->setRGBCameraIntrinsics(rgb_fx, rgb_fy, rgb_cx, rgb_cy);
	//interface->setDepthCameraIntrinsics(depth_fx, depth_fy, depth_cx, depth_cy);
}

void Realsense::registerCallbackFunction(std::function<void(const std::shared_ptr<const PointCloudType>& cloud)> lambda_f_cloud,
	std::function<void(const std::shared_ptr<rs2::video_frame>& image)> lambda_f_image)
{
	f_cloud = lambda_f_cloud;
	f_image = lambda_f_image;
}


bool Realsense::run() {
	initIntrinsics();
	grabber->setDeviceOptions(640, 480, 15);
	//grabber->setDeviceOptions(1280, 720, 30);
	connection_cloud = grabber->registerCallback(f_cloud);
	connection_image = grabber->registerCallback(f_image);
	grabber->start();

	return true;
}

void Realsense::stop() {
	grabber->stop();
	// Disconnect Callback Function
	if (connection_cloud.connected()) {
		connection_cloud.disconnect();
	}
	if (connection_image.connected()) {
		connection_image.disconnect();
	}
}
