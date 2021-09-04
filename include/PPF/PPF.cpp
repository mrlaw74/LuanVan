//#include "PPF.h"
//
//void getTransMatrixJointiPPF(double alpha, double a, double d, float theta, cv::Mat& transformMatrix)
//{
//	cv::Mat Rotz = (cv::Mat_<double>(4, 4) << cos(theta), -sin(theta), 0, 0,
//		sin(theta), cos(theta), 0, 0,
//		0, 0, 1, 0,
//		0, 0, 0, 1);
//	cv::Mat Rotx = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
//		0, cos(alpha), -sin(alpha), 0,
//		0, sin(alpha), cos(alpha), 0,
//		0, 0, 0, 1);
//	cv::Mat Transz = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
//		0, 1, 0, 0,
//		0, 0, 1, d,
//		0, 0, 0, 1);
//	cv::Mat Transx = (cv::Mat_<double>(4, 4) << 1, 0, 0, a,
//		0, 1, 0, 0,
//		0, 0, 1, 0,
//		0, 0, 0, 1);
//	transformMatrix = Rotz * Transz * Rotx * Transx;
//}
//
//void DescriptorPPF::doWork(const QString &parameter)
//{
//	QString result;
//	if (parameter == "loadMeshModelFile")
//	{
//		loadMeshModelFile();
//		result = "loadMeshModelFileSuccess";
//		emit resultReady(result);
//		return;
//	}
//	if (parameter == "prepareModelDescriptor")
//	{
//		prepareModelDescriptor();
//		result = "prepareSuccess";
//		emit resultReady(result);
//		return;
//	}
//	if (parameter == "detectObjects")
//	{
//		setRealSceneType();
//		_3D_Matching(); //Output ra ObjectTreals
//		
//		result = "detectObjectsSuccess";
//		emit resultReady(result);
//		return;
//	}
//	if (parameter == "simulationDetection")
//	{
//		setSimulationType();
//		_3D_Matching();
//		result = "simulationDetectionSuccess";
//		emit resultReady(result);
//		return;
//	}
//
//
//}
//
//	
//void DescriptorPPF::detect(const MyArray& pos)
//{
//	robotPos[0] = pos[0]; robotPos[1] = pos[1]; robotPos[2] = pos[2];
//	robotPos[3] = pos[3]; robotPos[4] = pos[4]; robotPos[5] = pos[5];
//	setRealSceneType();
//	_3D_Matching();
//	QString result = "SequenceDetectionSuccess";
//	emit resultReady(result);
//	return;
//}
//
//DescriptorPPF::DescriptorPPF() {}
//
//void DescriptorPPF::setSimulationType()
//{
//	isSimulation = true;
//}
//
//void DescriptorPPF::setRealSceneType()
//{
//	isSimulation = false;
//}
//
//std::string DescriptorPPF::getType()
//{
//	return type;
//}
//
//void DescriptorPPF::setMeshModelFileName(QString file_name)
//{
//	meshModelFileName = file_name;
//}
//
//
//void DescriptorPPF::loadMeshModelFile()
//{
//	std::string model_filename_ = meshModelFileName.toStdString();
//    //std::cout << "Loading mesh..." << std::endl;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr model_sampling(new pcl::PointCloud<pcl::PointXYZ>());
//	model.reset(new pcl::PointCloud<pcl::PointXYZ>());
//    //meshSampling(model_filename_, 1000000, 0.0005f, false, model_sampling);
//	meshSampling(model_filename_, 1000000, 0.0008f, false, model_sampling);
//
//    //------------------------- Calculate MEAM --------------------------------------------
//    std::vector<std::vector<float>> camera_pos(6);
//    pcl::PointXYZ minPt, maxPt, avgPt;
//
//    pcl::getMinMax3D(*model_sampling, minPt, maxPt);
//    avgPt.x = (minPt.x + maxPt.x) / 2;
//    avgPt.y = (minPt.y + maxPt.y) / 2;
//    avgPt.z = (minPt.z + maxPt.z) / 2;
//
//    float cube_length = std::max(maxPt.x - minPt.x, std::max(maxPt.y - minPt.y, maxPt.z - minPt.z));
//
//    minPt.x = avgPt.x - cube_length;
//    minPt.y = avgPt.y - cube_length;
//    minPt.z = avgPt.z - cube_length;
//    maxPt.x = avgPt.x + cube_length;
//    maxPt.y = avgPt.y + cube_length;
//    maxPt.z = avgPt.z + cube_length;
//
//    camera_pos[0] = { avgPt.x, minPt.y, avgPt.z };
//    camera_pos[1] = { maxPt.x, avgPt.y, avgPt.z };
//    camera_pos[2] = { avgPt.x, maxPt.y, avgPt.z };
//    camera_pos[3] = { minPt.x, avgPt.y, avgPt.z };
//    camera_pos[4] = { avgPt.x, avgPt.y, maxPt.z };
//    camera_pos[5] = { avgPt.x, avgPt.y, minPt.z };
//
//    std::cout << "Preparing Multiview Model....." << std::endl;
//
//    for (int i = 0; i < static_cast<int>(camera_pos.size()); ++i)
//    {
//        std::cout << "Preparing Viewpoint " << i << "....." << std::endl;
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_HPR(new pcl::PointCloud<pcl::PointXYZ>());
//        HPR(model_sampling, camera_pos[i], 3, cloud_xyz_HPR);
//
//        *model += *cloud_xyz_HPR;
//
//    }
//
//	// centering
//	Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
//	for (const auto& p : *(model)) sum_of_pos += p.getVector3fMap().cast<double>();
//
//	Eigen::Matrix4d transform_centering = Eigen::Matrix4d::Identity();
//	transform_centering.topRightCorner<3, 1>() = -sum_of_pos / model->size();
//
//	pcl::transformPointCloud(*model, *model, transform_centering);
//	pcl::transformPointCloud(*model, *model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0.7071, 0, -0.7071, 0));
//
//	std::cout << "Done Preparing Model....." << std::endl;
//}
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr DescriptorPPF::getModel()
//{
//	return model;
//}
//
//
//void DescriptorPPF::loadModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& model_) {
//
//	model = model_->makeShared();
//	std::cout << "Model Loaded!" << std::endl;
//}
//
//void DescriptorPPF::prepareModelDescriptor()
//{
//	double diameter_model = computeCloudDiameter(model);
//	std::cout << "Diameter : " << diameter_model << std::endl;
//
//	samp_rad = t_sampling * diameter_model;
//	norm_rad = 2 * samp_rad;
//	Lvoxel = 2 * samp_rad;
//
//	std::cout << "Preparing Model Descriptor....." << std::endl;
//	pcl::VoxelGrid<pcl::PointXYZ> vg;
//	vg.setInputCloud(model);
//	vg.setLeafSize(samp_rad, samp_rad, samp_rad);
//	vg.setDownsampleAllData(false);
//	vg.filter(*model_keypoints);
//
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	// Calculate all the normals of the entire surface
//	ne.setInputCloud(model_keypoints);
//	ne.setSearchSurface(model);
//	ne.setNumberOfThreads(8);
//	ne.setSearchMethod(tree);
//	ne.setRadiusSearch(norm_rad);
//	ne.compute(*normals);
//
//	pcl::concatenateFields(*model_keypoints, *normals, *model_keypoints_with_normals);
//
//
//	pcl::PointCloud<pcl::PPFSignature>::Ptr descriptors_PPF = pcl::PointCloud<pcl::PPFSignature>::Ptr(new pcl::PointCloud<pcl::PPFSignature>());
//	pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
//	ppf_estimator.setInputCloud(model_keypoints_with_normals);
//	ppf_estimator.setInputNormals(model_keypoints_with_normals);
//	ppf_estimator.compute(*descriptors_PPF);
//
//	ppf_hashmap_search->setInputFeatureCloud(descriptors_PPF);
//	
//	std::cout << "Done with Preparing Model Descriptor....." << std::endl;
//}
//
//void DescriptorPPF::storeLatestCloud(const PointCloudType::ConstPtr &cloud)
//{
//	latestCloud = cloud->makeShared();
//	//std::cout << "Cloud Update with Size " << latestCloud->points.size() << " ........." << std::endl;
//}
//
//bool DescriptorPPF::_3D_Matching()
//{
//	if (latestCloud->size() == 0)
//	{
//		//std::cout << "Cloud has no point to detect!" << std::endl;
//		return false;
//	}
//	tt.tic();
//	//Scene
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_scene = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr captured_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr passthroughed_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid_filtered_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_filtered_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointNormal>::Ptr scene_keypoints_with_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
//
//	mtx.lock();
//	pcl::copyPointCloud(*latestCloud, *captured_scene);
//	pcl::copyPointCloud(*latestCloud, *colored_scene);
//	mtx.unlock();
//
//	//std::cout << "3D Matching....." << std::endl;
//
//	captured_scene->is_dense = false;
//	std::shared_ptr<std::vector<int>> indices_nan(new std::vector<int>);
//	pcl::removeNaNFromPointCloud(*captured_scene, *captured_scene, *indices_nan);
//	captured_scene->is_dense = true;
//
//	//std::cout << "Cloud After Remove NaNs : " << captured_scene->points.size() << " ........." << std::endl;
//
//	// --------------------------------------------  Preprocess--------------------------------------------------------------
//	//tt.tic();
//	//passthrough(captured_scene, passthrough_limits, passthroughed_scene); // Get limited range point cloud
//	//cout << "passthrough in " << tt.toc() << " mseconds..." << std::endl;
//
//	//tt.tic();
//	//statisticalOutlinerRemoval(passthroughed_scene, 50, statistical_filtered_scene);// 50 k-neighbors noise removal
//	//cout << "statisticalOutlinerRemoval in " << tt.toc() << " seconds..." << std::endl;
//	pcl::visualization::PCLVisualizer::Ptr currentViewer = isSimulation?simulationViewer:detectObjectsViewer;
//	if (!isSimulation)
//	{
//
//		//cv::Mat T60 = cv::Mat::eye(cv::Size(4, 4), 6);
//		//for (int i = 0; i < 6; ++i)
//		//{
//		//	cv::Mat Tjointi;
//		//	getTransMatrixJointiPPF(alpha[i], a[i], d[i], robotPos[i] * M_PI / 180, Tjointi);
//		//	T60 *= Tjointi;
//		//}
//
//		//Eigen::Matrix4d eigenT60;
//		//cv::cv2eigen(T60, eigenT60);
//
//		//Eigen::Matrix4d eigenCamTend;
//		//cv::cv2eigen(camTend, eigenCamTend);
//
//		//Eigen::Matrix4f camTbase = (eigenT60 * eigenCamTend).cast<float>();
//		//pcl::transformPointCloud(*captured_scene, *passthroughed_scene, camTbase);
//
//
//		//std::vector<float> passthrough_lim = { 0.01, 1, 0.160, 0.260, -0.550, -0.285 };
//		//passthrough(passthroughed_scene, passthrough_lim, segmented_scene); // Get limited range point cloud
//		//pcl::transformPointCloud(*segmented_scene, *segmented_scene, camTbase.inverse());
//		//*scene = *segmented_scene; // get the preprocessed scene
//
//		
//
//		//sacsegmentation_extindices(captured_scene, 0.005, segmented_scene); // RANSAC Segmentation and remove biggest plane (table)
//		sacsegmentation_extindices(captured_scene, 0.005, segmented_scene); // RANSAC Segmentation and remove biggest plane (table)
//		//cout << "sacsegmentation_extindices in " << tt.toc() << " mseconds..." << std::endl;
//		*scene = *segmented_scene; // get the preprocessed scene
//		
//		//cout << "copied in " << tt.toc() << " mseconds..." << std::endl;
//	}
//	else {
//
//
//		tt.tic();
//		*scene = *captured_scene; // get the preprocessed scene
//		//cout << "copied in " << tt.toc() << " mseconds..." << std::endl;
//	}
//
//	if (scene->size() == 0)
//	{
//		//std::cout << "No point left in scene. Skipping this frame ..." << std::endl;
//		return false;
//	}
//
//	tt.tic();
//	// -----------------------------------------Voxel grid ------------------------------------------------------
//	pcl::VoxelGrid<pcl::PointXYZ> vg_;
//	vg_.setInputCloud(scene);
//	vg_.setLeafSize(0.001f, 0.001f, 0.001f);
//	vg_.setDownsampleAllData(true);
//	vg_.filter(*scene_keypoints);
//	statisticalOutlinerRemoval(scene_keypoints, 50, scene_keypoints);// 50 k-neighbors noise removal
//
//	pcl::VoxelGrid<pcl::PointXYZ> vg;
//	vg.setInputCloud(scene_keypoints);
//	vg.setLeafSize(samp_rad, samp_rad, samp_rad);
//	vg.setDownsampleAllData(true);
//	vg.filter(*scene_keypoints);
//
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	// Calculate all the normals of the entire surface
//	ne.setInputCloud(scene_keypoints);
//	ne.setSearchSurface(scene);
//	ne.setNumberOfThreads(8);
//	ne.setSearchMethod(tree);
//	ne.setRadiusSearch(norm_rad);
//	ne.compute(*normals);
//
//	pcl::concatenateFields(*scene_keypoints, *normals, *scene_keypoints_with_normals);
//	cout << "Preprocessed in " << tt.toc() << " mseconds..." << std::endl;
//
//	tt.tic();
//	// -------------------------------------------------B2B-TL MEAM ---------------------------------------------
//	pcl::MyPPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;
//	// set parameters for the PPF registration procedure
//	ppf_registration.setSceneReferencePointSamplingRate(scene_reference_point_sampling_rate);
//	ppf_registration.setSceneReferredPointSamplingRate(scene_referred_point_sampling_rate);
//	ppf_registration.setLvoxel(Lvoxel);
//	ppf_registration.setPositionClusteringThreshold(0.03f);
//	ppf_registration.setRotationClusteringThreshold(20.0f / 180.0f * float(M_PI));
//	ppf_registration.setSearchMethod(ppf_hashmap_search);
//	ppf_registration.setFullModel(model);
//	ppf_registration.setModelSamplingRadius(samp_rad);
//	ppf_registration.setInputSource(model_keypoints_with_normals);
//	ppf_registration.setInputTarget(scene_keypoints_with_normals);
//
//	typename pcl::MyPPFRegistration<pcl::PointNormal, pcl::PointNormal>::PoseWithVotesList results;
//	ppf_registration.computeFinalPoses(results);
//	cout << "PPF Scene Calculation in " << tt.toc() << " mseconds..." << std::endl;
//	
//	tt.tic();
//	//----------------------------------------------------------- ICP ----------------------------------------------------
//	std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> instances;
//	objectTransfromMatListFromCam.clear();
//	for (size_t results_i = 0; results_i < results.size(); ++results_i)
//	{
//		// Generates clouds for each instances found
//		pcl::PointCloud<pcl::PointXYZ>::Ptr instance(new pcl::PointCloud<pcl::PointXYZ>());
//		pcl::transformPointCloud(*model, *instance, results[results_i].pose);
//
//		std::vector<float> camera_pos = { 0, 0 ,0 };
//		HPR(instance, camera_pos, 3, instance);
//
//		pcl::VoxelGrid<pcl::PointXYZ> vg_icp;
//		vg_icp.setInputCloud(instance);
//		vg_icp.setLeafSize(samp_rad, samp_rad, samp_rad);
//		vg_icp.setDownsampleAllData(true);
//		vg_icp.filter(*instance);
//
//		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//		icp.setMaximumIterations(icp_max_iter_);
//		icp.setMaxCorrespondenceDistance(icp_corr_distance_);
//		icp.setInputTarget(scene_keypoints);
//		icp.setInputSource(instance);
//		pcl::PointCloud<pcl::PointXYZ>::Ptr registered_instance(new pcl::PointCloud<pcl::PointXYZ>);
//		icp.align(*registered_instance);
//
//		//std::cout << "Instance " << results_i << " ";
//		if (icp.hasConverged())
//		{
//			//std::cout << "Aligned!" << std::endl;
//		}
//		else
//		{
//			//std::cout << "Not Aligned!" << std::endl;
//		}
//
//		Eigen::Matrix4f transformation = icp.getFinalTransformation();
//		transformation = transformation * results[results_i].pose.matrix();
//
//		pcl::PointCloud<pcl::PointXYZ> Oz;
//		Oz.push_back(pcl::PointXYZ(0, 0, 0));
//		Oz.push_back(pcl::PointXYZ(0, 0, 0.05));
//		pcl::transformPointCloud(Oz, Oz, transformation);
//		if (Oz[1].z - Oz[0].z < 0)
//		{
//			Eigen::Matrix4f rotx180;
//			rotx180 << 1, 0, 0, 0,
//				0, -1, 0, 0,
//				0, 0, -1, 0,
//				0, 0, 0, 1;
//			transformation = transformation * rotx180;
//		}
//
//		Eigen::MatrixXf pointO(4, 1);
//		pointO << 0, 0, 0, 1;
//		pointO = transformation * pointO;
//		Eigen::MatrixXf pointOy(4, 1);
//		pointOy << 0, -1, 0, 1;
//		pointOy = transformation * pointOy;
//		Eigen::MatrixXf vectorOy(4, 1);
//		vectorOy = pointOy - pointO;
//		vectorOy(3, 0) = 0;
//
//		double dot = vectorOy(0,0) * 0 + vectorOy(1,0) * 1;
//		double det = vectorOy(0,0) * 1 - vectorOy(1,0) * 0;
//		double angle = atan2(det, dot);
//
//		Eigen::Matrix4f rotAroundZ;
//		rotAroundZ << cos(angle), -sin(angle), 0, 0,
//			sin(angle), cos(angle), 0, 0,
//			0, 0, 1, 0,
//			0, 0, 0, 1;
//		transformation =  transformation * rotAroundZ;
//
//
//		pcl::transformPointCloud(*model_keypoints, *instance, transformation);
//		instances.push_back(instance);
//
//		Eigen::Matrix4d objectTransfromMatFromCam = transformation.cast<double>();
//
//		std::cout << "Trans Mat From Cam: " << objectTransfromMatFromCam << std::endl;
//
//		objectTransfromMatListFromCam.push_back(objectTransfromMatFromCam);
//		//std::cout << "\nObjTcamList.size: \t" << objectTransfromMatListFromCam.size() << std::endl;
//	}
//
//
//	std::vector<bool> chosenPose(instances.size(), false);
//	if (objectTransfromMatListFromCam.size() > 0)
//	{
//		//std::cout << "1" << std::endl;
//		double lowest_z = objectTransfromMatListFromCam[0](2,3);
//		//std::cout << "1" << std::endl;
//		int lowestIndex = 0;
//		//std::cout << "1" << std::endl;
//		for (int i = 1; i < objectTransfromMatListFromCam.size(); ++i)
//		{
//			if (objectTransfromMatListFromCam[i](2, 3) < lowest_z)
//			{
//				lowest_z = objectTransfromMatListFromCam[i](2,3);
//				lowestIndex = i;
//			}
//			
//		}
//		//std::cout << "1" << std::endl;
//		chosenPose[lowestIndex] = true;
//	}
//
//
//
//	/*cout << "ICP in " << tt.toc() << " mseconds..." << std::endl;
//	cout << "instances size: " << instances.size()	 << std::endl;*/
//
//	//tt.tic();
//	// -------------------------------------------------------- Visualization --------------------------------------------------------
//
//
//	currentViewer->removeAllShapes();
//	currentViewer->removeAllPointClouds();
//
//	if (!isSimulation)
//	{
//		detectObjectsHandler->setInputCloud(colored_scene);
//		currentViewer->addPointCloud<pcl::PointXYZRGBA>(colored_scene, *detectObjectsHandler, "captured_scene");
//		currentViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "captured_scene");
//
//		//currentViewer->addPointCloud(scene, "captured_scene");
//	}
//	else
//	{
//		currentViewer->addPointCloud(captured_scene, "captured_scene");
//	}
//	
//
//	//currentViewer->addPointCloud<pcl::PointXYZ>(segmented_scene/*, *detectObjectsHandler*/, "segmented_scene");
//	//currentViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segmented_scene");
//	//Draw new Matched model-scene
//	for (std::size_t i = 0; i < instances.size(); ++i)
//	{
//		std::stringstream ss_instance;
//		ss_instance << "instance_" << i;
//
//		if (chosenPose[i])
//		{
//			CloudStyle pickedStyle = style_cyan;
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> instance_color_handler(instances[i], pickedStyle.r, pickedStyle.g, pickedStyle.b);
//			currentViewer->addPointCloud(instances[i], instance_color_handler, ss_instance.str());
//			currentViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pickedStyle.size, ss_instance.str());
//		}
//		else {
//			CloudStyle detectedStyle = style_green;
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> instance_color_handler(instances[i], detectedStyle.r, detectedStyle.g, detectedStyle.b);
//			currentViewer->addPointCloud(instances[i], instance_color_handler, ss_instance.str());
//			currentViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, detectedStyle.size, ss_instance.str());
//		}
//		
//		Eigen::Matrix4d transform = objectTransfromMatListFromCam[i];
//
//		pcl::PointCloud<pcl::PointXYZ> Oxyz;
//		Oxyz.push_back(pcl::PointXYZ(0, 0, 0));
//		Oxyz.push_back(pcl::PointXYZ(0.05, 0, 0));
//		Oxyz.push_back(pcl::PointXYZ(0, 0.05, 0));
//		Oxyz.push_back(pcl::PointXYZ(0, 0, 0.05));
//		pcl::transformPointCloud(Oxyz, Oxyz, transform);
//
//		currentViewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(Oxyz[0], Oxyz[1], 255, 0, 0, ss_instance.str() + "x");
//		currentViewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(Oxyz[0], Oxyz[2], 0, 255, 0, ss_instance.str() + "y");
//		currentViewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(Oxyz[0], Oxyz[3], 0, 0, 255, ss_instance.str() + "z");
//		currentViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str() + "x");
//		currentViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str() + "y");
//		currentViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str() + "z");
//
//		//Eigen::Matrix3d rotation;
//		//rotation << transform(0, 0), transform(0, 1), transform(0, 2),
//		//			transform(1, 0), transform(1, 1), transform(1, 2),
//		//			transform(2, 0), transform(2, 1), transform(2, 2);
//		//Eigen::Vector3d ea = rotation.eulerAngles(0, 1, 2); 
//		//cout << "to Euler angles:" << endl;
//		//cout << ea << endl << endl;
//	}
//	if (!isSimulation)
//		emit updateUiFromWorker("detectionViewer");
//	else
//		emit updateUiFromWorker("simulationViewer");
//	//cout << "Visualize in " << tt.toc() << " mseconds..." << std::endl;
//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
//
//	return true;
//}


#include "PPF.h"

void getTransMatrixJointiPPF(double alpha, double a, double d, float theta, cv::Mat& transformMatrix)
{
	cv::Mat Rotz = (cv::Mat_<double>(4, 4) << cos(theta), -sin(theta), 0, 0,
		sin(theta), cos(theta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	cv::Mat Rotx = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
		0, cos(alpha), -sin(alpha), 0,
		0, sin(alpha), cos(alpha), 0,
		0, 0, 0, 1);
	cv::Mat Transz = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, d,
		0, 0, 0, 1);
	cv::Mat Transx = (cv::Mat_<double>(4, 4) << 1, 0, 0, a,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	transformMatrix = Rotz * Transz * Rotx * Transx;
}

void DescriptorPPF::doWork(const QString &parameter)
{
	QString result;
	if (parameter == "loadMeshModelFile")
	{
		loadMeshModelFile();
		result = "loadMeshModelFileSuccess";
		emit resultReady(result);
		return;
	}
	if (parameter == "prepareModelDescriptor")
	{
		prepareModelDescriptor();
		result = "prepareSuccess";
		emit resultReady(result);
		return;
	}
	if (parameter == "detectObjects")
	{
		setRealSceneType();
		_3D_Matching();
		result = "detectObjectsSuccess";
		emit resultReady(result);
		return;
	}
	if (parameter == "simulationDetection")
	{
		setSimulationType();
		_3D_Matching();
		result = "simulationDetectionSuccess";
		emit resultReady(result);
		return;
	}


}


void DescriptorPPF::detect(const MyArray& pos)
{
	robotPos[0] = pos[0]; robotPos[1] = pos[1]; robotPos[2] = pos[2];
	robotPos[3] = pos[3]; robotPos[4] = pos[4]; robotPos[5] = pos[5];
	setRealSceneType();
	_3D_Matching();
	QString result = "SequenceDetectionSuccess";
	emit resultReady(result);
	return;
}

DescriptorPPF::DescriptorPPF() {}

void DescriptorPPF::setSimulationType()
{
	isSimulation = true;
}

void DescriptorPPF::setRealSceneType()
{
	isSimulation = false;
}

std::string DescriptorPPF::getType()
{
	return type;
}

void DescriptorPPF::setMeshModelFileName(QString file_name)
{
	meshModelFileName = file_name;
}


void DescriptorPPF::loadMeshModelFile()
{
	std::string model_filename_ = meshModelFileName.toStdString();
	//std::cout << "Loading mesh..." << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_sampling(new pcl::PointCloud<pcl::PointXYZ>());
	model.reset(new pcl::PointCloud<pcl::PointXYZ>());
	meshSampling(model_filename_, 1000000, 0.0005f, false, model_sampling);
	//meshSampling(model_filename_, 1000000, 0.0005f, false, model_sampling);

	//------------------------- Calculate MEAM --------------------------------------------
	std::vector<std::vector<float>> camera_pos(6);
	pcl::PointXYZ minPt, maxPt, avgPt;

	pcl::getMinMax3D(*model_sampling, minPt, maxPt);
	avgPt.x = (minPt.x + maxPt.x) / 2;
	avgPt.y = (minPt.y + maxPt.y) / 2;
	avgPt.z = (minPt.z + maxPt.z) / 2;

	float cube_length = std::max(maxPt.x - minPt.x, std::max(maxPt.y - minPt.y, maxPt.z - minPt.z));

	minPt.x = avgPt.x - cube_length;
	minPt.y = avgPt.y - cube_length;
	minPt.z = avgPt.z - cube_length;
	maxPt.x = avgPt.x + cube_length;
	maxPt.y = avgPt.y + cube_length;
	maxPt.z = avgPt.z + cube_length;

	camera_pos[0] = { avgPt.x, minPt.y, avgPt.z };
	camera_pos[1] = { maxPt.x, avgPt.y, avgPt.z };
	camera_pos[2] = { avgPt.x, maxPt.y, avgPt.z };
	camera_pos[3] = { minPt.x, avgPt.y, avgPt.z };
	camera_pos[4] = { avgPt.x, avgPt.y, maxPt.z };
	camera_pos[5] = { avgPt.x, avgPt.y, minPt.z };

	//std::cout << "Preparing Multiview Model....." << std::endl;

	for (int i = 0; i < static_cast<int>(camera_pos.size()); ++i)
	{
		//std::cout << "Preparing Viewpoint " << i << "....." << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_HPR(new pcl::PointCloud<pcl::PointXYZ>());
		HPR(model_sampling, camera_pos[i], 3, cloud_xyz_HPR);

		*model += *cloud_xyz_HPR;

	}

	// centering
	Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
	for (const auto& p : *(model)) sum_of_pos += p.getVector3fMap().cast<double>();

	Eigen::Matrix4d transform_centering = Eigen::Matrix4d::Identity();
	transform_centering.topRightCorner<3, 1>() = -sum_of_pos / model->size();

	pcl::transformPointCloud(*model, *model, transform_centering);
	pcl::transformPointCloud(*model, *model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0.7071, 0, -0.7071, 0));

	//std::cout << "Done Preparing Model....." << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DescriptorPPF::getModel()
{
	return model;
}


void DescriptorPPF::loadModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& model_) {

	model = model_->makeShared();
	//std::cout << "Model Loaded!" << std::endl;
}

void DescriptorPPF::prepareModelDescriptor()
{
	double diameter_model = computeCloudDiameter(model);
	//std::cout << "Diameter : " << diameter_model << std::endl;

	samp_rad = t_sampling * diameter_model;
	norm_rad = 2 * samp_rad;
	Lvoxel = 2 * samp_rad;

	//std::cout << "Preparing Model Descriptor....." << std::endl;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(model);
	vg.setLeafSize(samp_rad, samp_rad, samp_rad);
	vg.setDownsampleAllData(false);
	vg.filter(*model_keypoints);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// Calculate all the normals of the entire surface
	ne.setInputCloud(model_keypoints);
	ne.setSearchSurface(model);
	ne.setNumberOfThreads(8);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(norm_rad);
	ne.compute(*normals);

	pcl::concatenateFields(*model_keypoints, *normals, *model_keypoints_with_normals);


	pcl::PointCloud<pcl::PPFSignature>::Ptr descriptors_PPF = pcl::PointCloud<pcl::PPFSignature>::Ptr(new pcl::PointCloud<pcl::PPFSignature>());
	pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
	ppf_estimator.setInputCloud(model_keypoints_with_normals);
	ppf_estimator.setInputNormals(model_keypoints_with_normals);
	ppf_estimator.compute(*descriptors_PPF);

	ppf_hashmap_search->setInputFeatureCloud(descriptors_PPF);

	//std::cout << "Done with Preparing Model Descriptor....." << std::endl;
}

void DescriptorPPF::storeLatestCloud(const PointCloudType::ConstPtr &cloud)
{
	latestCloud = cloud->makeShared();
	//std::cout << "Cloud Update with Size " << latestCloud->points.size() << " ........." << std::endl;
}

bool DescriptorPPF::_3D_Matching()
{
	if (latestCloud->size() == 0)
	{
		//std::cout << "Cloud has no point to detect!" << std::endl;
		return false;
	}
	tt.tic();
	//Scene
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_scene = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr captured_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr passthroughed_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid_filtered_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_filtered_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointNormal>::Ptr scene_keypoints_with_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());

	mtx.lock();
	pcl::copyPointCloud(*latestCloud, *captured_scene);
	pcl::copyPointCloud(*latestCloud, *colored_scene);
	mtx.unlock();

	//std::cout << "3D Matching....." << std::endl;

	captured_scene->is_dense = false;
	std::shared_ptr<std::vector<int>> indices_nan(new std::vector<int>);
	pcl::removeNaNFromPointCloud(*captured_scene, *captured_scene, *indices_nan);
	captured_scene->is_dense = true;

	//std::cout << "Cloud After Remove NaNs : " << captured_scene->points.size() << " ........." << std::endl;

	// --------------------------------------------  Preprocess--------------------------------------------------------------
	//tt.tic();
	//passthrough(captured_scene, passthrough_limits, passthroughed_scene); // Get limited range point cloud
	//cout << "passthrough in " << tt.toc() << " mseconds..." << std::endl;

	//tt.tic();
	//statisticalOutlinerRemoval(passthroughed_scene, 50, statistical_filtered_scene);// 50 k-neighbors noise removal
	//cout << "statisticalOutlinerRemoval in " << tt.toc() << " seconds..." << std::endl;
	pcl::visualization::PCLVisualizer::Ptr currentViewer = isSimulation ? simulationViewer : detectObjectsViewer;
	if (!isSimulation)
	{

		//cv::Mat T60 = cv::Mat::eye(cv::Size(4, 4), 6);
		//for (int i = 0; i < 6; ++i)
		//{
		//	cv::Mat Tjointi;
		//	getTransMatrixJointiPPF(alpha[i], a[i], d[i], robotPos[i] * M_PI / 180, Tjointi);
		//	T60 *= Tjointi;
		//}

		//Eigen::Matrix4d eigenT60;
		//cv::cv2eigen(T60, eigenT60);

		//Eigen::Matrix4d eigenCamTend;
		//cv::cv2eigen(camTend, eigenCamTend);

		//Eigen::Matrix4f camTbase = (eigenT60 * eigenCamTend).cast<float>();
		//pcl::transformPointCloud(*captured_scene, *passthroughed_scene, camTbase);


		//std::vector<float> passthrough_lim = { 0.01, 1, 0.160, 0.260, -0.550, -0.285 };
		//passthrough(passthroughed_scene, passthrough_lim, segmented_scene); // Get limited range point cloud
		//pcl::transformPointCloud(*segmented_scene, *segmented_scene, camTbase.inverse());
		//*scene = *segmented_scene; // get the preprocessed scene



		sacsegmentation_extindices(captured_scene, 0.005, segmented_scene); // RANSAC Segmentation and remove biggest plane (table)
		cout << "sacsegmentation_extindices in " << tt.toc() << " mseconds..." << std::endl;
		*scene = *segmented_scene; // get the preprocessed scene

		//cout << "copied in " << tt.toc() << " mseconds..." << std::endl;
	}
	else {


		tt.tic();
		*scene = *captured_scene; // get the preprocessed scene
		//cout << "copied in " << tt.toc() << " mseconds..." << std::endl;
	}

	if (scene->size() == 0)
	{
		//std::cout << "No point left in scene. Skipping this frame ..." << std::endl;
		return false;
	}

	tt.tic();
	// -----------------------------------------Voxel grid ------------------------------------------------------
	pcl::VoxelGrid<pcl::PointXYZ> vg_;
	vg_.setInputCloud(scene);
	vg_.setLeafSize(0.001f, 0.001f, 0.001f);
	vg_.setDownsampleAllData(true);
	vg_.filter(*scene_keypoints);
	statisticalOutlinerRemoval(scene_keypoints, 50, scene_keypoints);// 50 k-neighbors noise removal

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(scene_keypoints);
	vg.setLeafSize(samp_rad, samp_rad, samp_rad);
	vg.setDownsampleAllData(true);
	vg.filter(*scene_keypoints);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// Calculate all the normals of the entire surface
	ne.setInputCloud(scene_keypoints);
	ne.setSearchSurface(scene);
	ne.setNumberOfThreads(8);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(norm_rad);
	ne.compute(*normals);

	pcl::concatenateFields(*scene_keypoints, *normals, *scene_keypoints_with_normals);
	cout << "Preprocessed in " << tt.toc() << " mseconds..." << std::endl;

	tt.tic();
	// -------------------------------------------------B2B-TL MEAM ---------------------------------------------
	pcl::MyPPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;
	// set parameters for the PPF registration procedure
	ppf_registration.setSceneReferencePointSamplingRate(scene_reference_point_sampling_rate);
	ppf_registration.setSceneReferredPointSamplingRate(scene_referred_point_sampling_rate);
	ppf_registration.setLvoxel(Lvoxel);
	ppf_registration.setPositionClusteringThreshold(0.03f);
	ppf_registration.setRotationClusteringThreshold(20.0f / 180.0f * float(M_PI));
	ppf_registration.setSearchMethod(ppf_hashmap_search);
	ppf_registration.setFullModel(model);
	ppf_registration.setModelSamplingRadius(samp_rad);
	ppf_registration.setInputSource(model_keypoints_with_normals);
	ppf_registration.setInputTarget(scene_keypoints_with_normals);

	typename pcl::MyPPFRegistration<pcl::PointNormal, pcl::PointNormal>::PoseWithVotesList results;
	ppf_registration.computeFinalPoses(results);
	cout << "PPF MEAM Scene Calculation in " << tt.toc() << " mseconds..." << std::endl;

	tt.tic();
	//----------------------------------------------------------- ICP ----------------------------------------------------
	std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> instances;
	objectTransfromMatListFromCam.clear();
	for (size_t results_i = 0; results_i < results.size(); ++results_i)
	{
		// Generates clouds for each instances found
		pcl::PointCloud<pcl::PointXYZ>::Ptr instance(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*model, *instance, results[results_i].pose);

		std::vector<float> camera_pos = { 0, 0 ,0 };
		HPR(instance, camera_pos, 3, instance);

		pcl::VoxelGrid<pcl::PointXYZ> vg_icp;
		vg_icp.setInputCloud(instance);
		vg_icp.setLeafSize(samp_rad, samp_rad, samp_rad);
		vg_icp.setDownsampleAllData(true);
		vg_icp.filter(*instance);

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaximumIterations(icp_max_iter_);
		icp.setMaxCorrespondenceDistance(icp_corr_distance_);
		icp.setInputTarget(scene_keypoints);
		icp.setInputSource(instance);
		pcl::PointCloud<pcl::PointXYZ>::Ptr registered_instance(new pcl::PointCloud<pcl::PointXYZ>);
		icp.align(*registered_instance);

		//std::cout << "Instance " << results_i << " ";
		if (icp.hasConverged())
		{
			//std::cout << "Aligned!" << std::endl;
		}
		else
		{
			//std::cout << "Not Aligned!" << std::endl;
		}

		Eigen::Matrix4f transformation = icp.getFinalTransformation();
		transformation = transformation * results[results_i].pose.matrix();

		pcl::PointCloud<pcl::PointXYZ> Oz;
		Oz.push_back(pcl::PointXYZ(0, 0, 0));
		Oz.push_back(pcl::PointXYZ(0, 0, 0.05));
		pcl::transformPointCloud(Oz, Oz, transformation);
		if (Oz[1].z - Oz[0].z < 0)
		{
			Eigen::Matrix4f rotx180;
			rotx180 << 1, 0, 0, 0,
				0, -1, 0, 0,
				0, 0, -1, 0,
				0, 0, 0, 1;
			transformation = transformation * rotx180;
		}

		Eigen::MatrixXf pointO(4, 1);
		pointO << 0, 0, 0, 1;
		pointO = transformation * pointO;
		Eigen::MatrixXf pointOy(4, 1);
		pointOy << 0, -1, 0, 1;
		pointOy = transformation * pointOy;
		Eigen::MatrixXf vectorOy(4, 1);
		vectorOy = pointOy - pointO;
		vectorOy(3, 0) = 0;

		double dot = vectorOy(0, 0) * 0 + vectorOy(1, 0) * 1;
		double det = vectorOy(0, 0) * 1 - vectorOy(1, 0) * 0;
		double angle = atan2(det, dot);

		Eigen::Matrix4f rotAroundZ;
		rotAroundZ << cos(angle), -sin(angle), 0, 0,
			sin(angle), cos(angle), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		transformation = transformation * rotAroundZ;


		pcl::transformPointCloud(*model_keypoints, *instance, transformation);
		instances.push_back(instance);

		Eigen::Matrix4d objectTransfromMatFromCam = transformation.cast<double>();

		//std::cout << "Trans Mat From Cam: " << objectTransfromMatFromCam << std::endl;

		objectTransfromMatListFromCam.push_back(objectTransfromMatFromCam);
	}

	std::vector<bool> chosenPose(instances.size(), false);
	/*if (objectTransfromMatListFromCam.size() > 0)
	{
		
		double lowest_z = objectTransfromMatListFromCam[0](2, 3);
		int lowestIndex = 0;
		for (int i = 1; i < objectTransfromMatListFromCam.size(); ++i)
		{
			if (objectTransfromMatListFromCam[i](2, 3) < lowest_z)
			{
				lowest_z = objectTransfromMatListFromCam[i](2, 3);
				lowestIndex = i;
			}

		}
		chosenPose[lowestIndex] = true;
	}*/

	if (objectTransfromMatListFromCam.size() > 0)
	{

		double lowest_z = abs(objectTransfromMatListFromCam[0](2, 0));
		int lowestIndex = 0;
		for (int i = 1; i < objectTransfromMatListFromCam.size() - 1; ++i)
		{
			if (abs(objectTransfromMatListFromCam[i](2, 0)) > lowest_z)
			{
				lowest_z = objectTransfromMatListFromCam[i](2, 0);
				lowestIndex = i;
			}
		}
		chosenPose[lowestIndex] = true;
	}

	cout << "ICP in " << tt.toc() << " mseconds..." << std::endl;
	cout << "instances size: " << instances.size() << std::endl;

	//tt.tic();
	// -------------------------------------------------------- Visualization --------------------------------------------------------


	currentViewer->removeAllShapes();
	currentViewer->removeAllPointClouds();

	if (!isSimulation)
	{
		detectObjectsHandler->setInputCloud(colored_scene);
		currentViewer->addPointCloud<pcl::PointXYZRGBA>(colored_scene, *detectObjectsHandler, "captured_scene");
		currentViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "captured_scene");

		//currentViewer->addPointCloud(scene, "captured_scene");
	}
	else
	{
		currentViewer->addPointCloud(captured_scene, "captured_scene");
	}


	//currentViewer->addPointCloud<pcl::PointXYZ>(segmented_scene/*, *detectObjectsHandler*/, "segmented_scene");
	//currentViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segmented_scene");
	//Draw new Matched model-scene
	for (std::size_t i = 0; i < instances.size(); ++i)
	{
		std::stringstream ss_instance;
		ss_instance << "instance_" << i;

		if (chosenPose[i])
		{
			CloudStyle pickedStyle = style_cyan;
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> instance_color_handler(instances[i], pickedStyle.r, pickedStyle.g, pickedStyle.b);
			currentViewer->addPointCloud(instances[i], instance_color_handler, ss_instance.str());
			currentViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pickedStyle.size, ss_instance.str());
		}
		else {
			CloudStyle detectedStyle = style_green;
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> instance_color_handler(instances[i], detectedStyle.r, detectedStyle.g, detectedStyle.b);
			currentViewer->addPointCloud(instances[i], instance_color_handler, ss_instance.str());
			currentViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, detectedStyle.size, ss_instance.str());
		}

		Eigen::Matrix4d transform = objectTransfromMatListFromCam[i];

		pcl::PointCloud<pcl::PointXYZ> Oxyz;
		Oxyz.push_back(pcl::PointXYZ(0, 0, 0));
		Oxyz.push_back(pcl::PointXYZ(0.05, 0, 0));
		Oxyz.push_back(pcl::PointXYZ(0, 0.05, 0));
		Oxyz.push_back(pcl::PointXYZ(0, 0, 0.05));
		pcl::transformPointCloud(Oxyz, Oxyz, transform);

		currentViewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(Oxyz[0], Oxyz[1], 255, 0, 0, ss_instance.str() + "x");
		currentViewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(Oxyz[0], Oxyz[2], 0, 255, 0, ss_instance.str() + "y");
		currentViewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(Oxyz[0], Oxyz[3], 0, 0, 255, ss_instance.str() + "z");
		currentViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str() + "x");
		currentViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str() + "y");
		currentViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str() + "z");

		//Eigen::Matrix3d rotation;
		//rotation << transform(0, 0), transform(0, 1), transform(0, 2),
		//			transform(1, 0), transform(1, 1), transform(1, 2),
		//			transform(2, 0), transform(2, 1), transform(2, 2);
		//Eigen::Vector3d ea = rotation.eulerAngles(0, 1, 2); 
		//cout << "to Euler angles:" << endl;
		//cout << ea << endl << endl;
	}
	if (!isSimulation)
		emit updateUiFromWorker("detectionViewer");
	else
		emit updateUiFromWorker("simulationViewer");
	//cout << "Visualize in " << tt.toc() << " mseconds..." << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	return true;
}