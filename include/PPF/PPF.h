//#ifndef PPF
//#define PPF
////PCL Library
//#include <pcl/console/time.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/registration/icp.h>
//
////Qt Library
//#include <QObject>
//#include <QThread>
//#include <QVector>
//
////User Library
//#include "pclFunction.h"
//#include "meshSampling.h"
//#include "HPR.h"
//#include "MyPPFRegistration.hpp"
////#include "NachiSocket.h"
//
////OpenCV Library
//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"
//#include <opencv2/core/eigen.hpp>
//#include <opencv2/opencv.hpp>
//
//class DescriptorPPF : public QObject
//{
//	Q_OBJECT
//
//public Q_SLOTS:
//	void doWork(const QString &parameter);
//	void detect(const MyArray& pos);
//
//Q_SIGNALS:
//	void resultReady(const QString &result);
//	void updateUiFromWorker(const QString &);
//
//private:
//	//IO
//	bool isSimulation = false;
//	QString meshModelFileName;
//	std::mutex mtx;
//	float robotPos[6] = { -68.19, 73.48, -14.79, 0, -59.69 , 21.81 };
//	
//
//	std::vector<double> alpha{ M_PI / 2, 0, M_PI / 2, -M_PI / 2, M_PI / 2, 0 };
//	std::vector<double> a{ 0.05, 0.33, 0.045, 0, 0, 0 };
//	std::vector<double> d{ 0.345, 0, 0, 0.340, 0, 0.083 };
//	float theta[6] = { -80.63, 68.53, -5, 0.34, -63.5, -82.76 };
//
//	PointCloudType::Ptr latestCloud = PointCloudType::Ptr(new PointCloudType());
//
//	//Algorithm params
//	double t_sampling = 0.03; // 0.04
//	float samp_rad;
//	float norm_rad;
//	double Lvoxel;
//	bool show_FP = true;
//
//	std::vector<float> passthrough_limits = { -0.5, 0.5, -0.5, 0.5, -0.1, 1.0 };
//	float angle_discretization_step = 12.0f / 180.0f * float(M_PI);
//	float distance_discretization_step = 0.005f;
//	int scene_reference_point_sampling_rate = 10;
//	int scene_referred_point_sampling_rate = 5;
//	float hv_dist_thresh = distance_discretization_step;
//	size_t Npv = 4;
//
//	int icp_max_iter_ = 1000;
//	float icp_corr_distance_ = 0.01f;
//
//	std::string type = "PPF";
//
//	// Model 
//	pcl::PointCloud<pcl::PointXYZ>::Ptr model = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud <pcl::PointNormal>::Ptr model_keypoints_with_normals = pcl::PointCloud <pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>());
//
//	pcl::PointCloud<PointXYZTangent>::Ptr off_scene_model_keypoints_with_normals = pcl::PointCloud<PointXYZTangent>::Ptr(new pcl::PointCloud<PointXYZTangent>());
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	std::shared_ptr<pcl::MyPPFHashMapSearch> ppf_hashmap_search = std::make_shared<pcl::MyPPFHashMapSearch>(angle_discretization_step, distance_discretization_step);
//
//	
//
//	//Others
//	int prev_instances_size = 0; // for removing previous cloud
//	pcl::console::TicToc tt; // Tictoc for process-time calculation
//
//	
//public:
//	//std::shared_ptr<NachiSocket> soc;
//	cv::Mat camTend = cv::Mat::zeros(cv::Size(4,4), 6);
//	//Result
//	std::vector<Eigen::Matrix4d> objectTransfromMatListFromCam;
//
//	std::shared_ptr<pcl::visualization::PointCloudColorHandler<PointType>> detectObjectsHandler;
//	pcl::visualization::PCLVisualizer::Ptr detectObjectsViewer;
//
//	pcl::visualization::PCLVisualizer::Ptr simulationViewer;
//
//	DescriptorPPF();
//
//	void setSimulationType();
//	void setRealSceneType();
//	std::string getType();
//	void setMeshModelFileName(QString file_name);
//	void loadMeshModelFile();
//	pcl::PointCloud<pcl::PointXYZ>::Ptr getModel();
//	void loadModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& model_);
//	void prepareModelDescriptor();
//	void storeLatestCloud(const PointCloudType::ConstPtr &cloud);
//	bool _3D_Matching();
//};
//
//
//
//#endif

#ifndef PPF
#define PPF
//PCL Library
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>

//Qt Library
#include <QObject>
#include <QThread>
#include <QVector>

//User Library
#include "pclFunction.h"
#include "meshSampling.h"
#include "HPR.h"
#include "MyPPFRegistration.hpp"
//#include "NachiSocket.h"

//OpenCV Library
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

class DescriptorPPF : public QObject
{
	Q_OBJECT

public Q_SLOTS:
	void doWork(const QString &parameter);
	void detect(const MyArray& pos);

Q_SIGNALS:
	void resultReady(const QString &result);
	void updateUiFromWorker(const QString &);

private:
	//IO
	bool isSimulation = false;
	QString meshModelFileName;
	std::mutex mtx;
	float robotPos[6] = { -68.19, 73.48, -14.79, 0, -59.69 , 21.81 };


	std::vector<double> alpha{ M_PI / 2, 0, M_PI / 2, -M_PI / 2, M_PI / 2, 0 };
	std::vector<double> a{ 0.05, 0.33, 0.045, 0, 0, 0 };
	std::vector<double> d{ 0.345, 0, 0, 0.340, 0, 0.083 };
	float theta[6] = { 0, M_PI / 2, 0, 0 , 3 * M_PI / 2, 0 };

	PointCloudType::Ptr latestCloud = PointCloudType::Ptr(new PointCloudType());

	//Algorithm params
	double t_sampling = 0.04;
	float samp_rad;
	float norm_rad;
	double Lvoxel;
	bool show_FP = true;

	std::vector<float> passthrough_limits = { -0.5, 0.5, -0.5, 0.5, -0.1, 1.0 };
	float angle_discretization_step = 12.0f / 180.0f * float(M_PI);
	float distance_discretization_step = 0.005f;
	int scene_reference_point_sampling_rate = 10;
	int scene_referred_point_sampling_rate = 5;
	float hv_dist_thresh = distance_discretization_step;
	size_t Npv = 4;

	int icp_max_iter_ = 1000;
	float icp_corr_distance_ = 0.01f;

	std::string type = "PPF";

	// Model 
	pcl::PointCloud<pcl::PointXYZ>::Ptr model = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud <pcl::PointNormal>::Ptr model_keypoints_with_normals = pcl::PointCloud <pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>());

	pcl::PointCloud<PointXYZTangent>::Ptr off_scene_model_keypoints_with_normals = pcl::PointCloud<PointXYZTangent>::Ptr(new pcl::PointCloud<PointXYZTangent>());

	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	std::shared_ptr<pcl::MyPPFHashMapSearch> ppf_hashmap_search = std::make_shared<pcl::MyPPFHashMapSearch>(angle_discretization_step, distance_discretization_step);



	//Others
	int prev_instances_size = 0; // for removing previous cloud
	pcl::console::TicToc tt; // Tictoc for process-time calculation


public:
	//std::shared_ptr<NachiSocket> soc;
	cv::Mat camTend = cv::Mat::zeros(cv::Size(4, 4), 6);
	//Result
	std::vector<Eigen::Matrix4d> objectTransfromMatListFromCam;

	std::shared_ptr<pcl::visualization::PointCloudColorHandler<PointType>> detectObjectsHandler;
	pcl::visualization::PCLVisualizer::Ptr detectObjectsViewer;

	pcl::visualization::PCLVisualizer::Ptr simulationViewer;

	DescriptorPPF();

	void setSimulationType();
	void setRealSceneType();
	std::string getType();
	void setMeshModelFileName(QString file_name);
	void loadMeshModelFile();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getModel();
	void loadModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& model_);
	void prepareModelDescriptor();
	void storeLatestCloud(const PointCloudType::ConstPtr &cloud);
	bool _3D_Matching();
};



#endif