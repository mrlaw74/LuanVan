#include "pclviewer.h"
#include "ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
	vtkOutputWindow::GlobalWarningDisplayOff(); // This is a global flag that controls whether any debug, warning or error messages are displayed.
    ui->setupUi(this);
    this->setWindowTitle ("Luan Van Tot Nghiep - Nguyen Van Luat");

    // Setup the cloud pointer
	model.reset(new pcl::PointCloud<pcl::PointXYZ>());
	simulationCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    realsenseCloud.reset(new pcl::PointCloud<PointType>());
    removeBackgroundCloud.reset(new pcl::PointCloud<PointType>());
    detectObjectsCloud.reset(new pcl::PointCloud<PointType>());

    //Set up QLabel Image window
    cv::Mat blank = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
    QImage QBlank = QImage((uchar*) blank.data, blank.cols, blank.rows, blank.step, QImage::Format_RGB888);
    ui->RealsenseImageLabel->setPixmap(QPixmap::fromImage(QBlank));

    // Set up the QVTK window
    modelViewer.reset (new pcl::visualization::PCLVisualizer ("Model viewer", false));
    ui->qvtkWidgetModel->SetRenderWindow (modelViewer->getRenderWindow ());
    modelViewer->setupInteractor (ui->qvtkWidgetModel->GetInteractor (), ui->qvtkWidgetModel->GetRenderWindow ());
    ui->qvtkWidgetModel->update ();

	simulationViewer.reset (new pcl::visualization::PCLVisualizer ("Simulation viewer", false));
    ui->qvtkWidgetSimulation->SetRenderWindow (simulationViewer->getRenderWindow ());
    simulationViewer->setupInteractor (ui->qvtkWidgetSimulation->GetInteractor (), ui->qvtkWidgetSimulation->GetRenderWindow ());
    ui->qvtkWidgetSimulation->update ();

    realsenseViewer.reset (new pcl::visualization::PCLVisualizer ("Realsense 3D viewer", false));
    ui->qvtkWidgetCamera3D->SetRenderWindow (realsenseViewer->getRenderWindow ());
    realsenseViewer->setupInteractor (ui->qvtkWidgetCamera3D->GetInteractor (), ui->qvtkWidgetCamera3D->GetRenderWindow ());
    ui->qvtkWidgetCamera3D->update ();

    removeBackgroundViewer.reset (new pcl::visualization::PCLVisualizer ("Remove Background viewer", false));
    ui->qvtkWidgetRemoveBackground->SetRenderWindow (removeBackgroundViewer->getRenderWindow ());
    removeBackgroundViewer->setupInteractor (ui->qvtkWidgetRemoveBackground->GetInteractor (), ui->qvtkWidgetRemoveBackground->GetRenderWindow ());
    ui->qvtkWidgetRemoveBackground->update ();

    detectObjectsViewer.reset (new pcl::visualization::PCLVisualizer ("Detect Objects viewer", false));
    ui->qvtkWidgetObjects->SetRenderWindow (detectObjectsViewer->getRenderWindow ());
    detectObjectsViewer->setupInteractor (ui->qvtkWidgetObjects->GetInteractor (), ui->qvtkWidgetObjects->GetRenderWindow ());
    ui->qvtkWidgetObjects->update ();

    // Connect buttons and the function
    connect(ui->pushButton_BrowseModel, SIGNAL(clicked()), this, SLOT(browseModelButtonPressed()));
    connect(ui->pushButton_BrowseTransformMatrix, SIGNAL(clicked()), this, SLOT(browseTransformMatrixButtonPressed()));
    connect(ui->pushButton_StartSimulation, SIGNAL(clicked()), this, SLOT(startSimulationButtonPressed()));
    connect(ui->pushButton_SimulationDetectObjects, SIGNAL(clicked()), this, SLOT(simulationDetectObjectsButtonPressed()));
    connect(ui->pushButton_ConnectToNachiSocket, SIGNAL(clicked()), this, SLOT(connectToNachiSocketButtonPressed()));
    connect(ui->pushButton_StartRealsenseCamera, SIGNAL(clicked()), this, SLOT(startRealsenseCameraButtonPressed()));
    connect(ui->pushButton_RemoveBackground, SIGNAL(clicked()), this, SLOT(removeBackgroundButtonPressed()));
    connect(ui->pushButton_DetectObjects, SIGNAL(clicked()), this, SLOT(detectObjectsButtonPressed()));
    connect(ui->pushButton_MoveToCamera0, SIGNAL(clicked()), this, SLOT(moveToCamera0ButtonPressed()));
    connect(ui->pushButton_Pick1stObject, SIGNAL(clicked()), this, SLOT(pick1stObjectButtonPressed()));
    connect(ui->pushButton_StartSequence, SIGNAL(clicked()), this, SLOT(startSequenceButtonPressed()));

	//Initialize viewer
	modelHandler->setInputCloud(model);
    modelViewer->addPointCloud (model, *modelHandler, "model");
	modelViewer->addCoordinateSystem(0.05);
    ui->qvtkWidgetModel->update ();

	simulationHandler->setInputCloud(simulationCloud);
    simulationViewer->addPointCloud (simulationCloud, *simulationHandler, "simulation");
    ui->qvtkWidgetSimulation->update ();

    realsense3DHandler->setInputCloud(realsenseCloud);
    realsenseViewer->addPointCloud (realsenseCloud, *realsense3DHandler, "Realsense 3D");
    realsenseViewer->setCameraPosition(0, 0, -0.3,      0, 0, 0.4,     0, -1, 0);
    ui->qvtkWidgetCamera3D->update ();

    removeBackgroundHandler->setInputCloud(removeBackgroundCloud);
    removeBackgroundViewer->addPointCloud (removeBackgroundCloud, *removeBackgroundHandler, "Remove Background");
    removeBackgroundViewer->setCameraPosition(0, 0, -0.3,      0, 0, 0.4,     0, -1, 0);
    ui->qvtkWidgetRemoveBackground->update ();

    detectObjectsHandler->setInputCloud(detectObjectsCloud);
    detectObjectsViewer->addPointCloud (detectObjectsCloud, *detectObjectsHandler, "Detect Objects");
    detectObjectsViewer->setCameraPosition(0, 0, -0.3,      0, 0, 0.4,     0, -1, 0);
	detectObjectsViewer->addCoordinateSystem(0.05);
    ui->qvtkWidgetObjects->update ();

	//Setup Detection Controller
	detectController = new DetectionController();
	connect(detectController, SIGNAL(updateUiFromController(QString)), this, SLOT(updateUi(QString)));
	detectController->ppfWorker->detectObjectsHandler = detectObjectsHandler;
	detectController->ppfWorker->detectObjectsViewer = detectObjectsViewer;
	detectController->ppfWorker->simulationViewer = simulationViewer;

	qRegisterMetaType<MyArray>("MyArray");
	connect(detectController->robotWorker, SIGNAL(operateDetection(MyArray)), this, SLOT(doWork(MyArray)));
	detectController->PPFWorkerThread.start();

	detectController->simulationWorker->simulationViewer = simulationViewer;
	detectController->simulationWorkerThread.start();

	detectController->robotWorkerThread.start();
}

void
PCLViewer::browseModelButtonPressed ()
{
    QString filter = "STL file (*.STL) ;; ply file (*.ply) ;; obj file (*.obj)";
    QString file_name = QFileDialog::getOpenFileName(this, "Open Model File", QDir::homePath(), filter);
    if (file_name.isEmpty() || file_name.isNull())
    {
        return;
    }

	ui->pushButton_BrowseModel->setEnabled(false);

	detectController->ppfWorker->setMeshModelFileName(file_name);
	emit detectController->operate("loadMeshModelFile");
}

void
PCLViewer::browseTransformMatrixButtonPressed()
{
    QString filter = "dat file (*.dat)";
    QString file_name = QFileDialog::getOpenFileName(this, "Open Model File", QDir::homePath(), filter);
    if (file_name.isEmpty() || file_name.isNull())
    {
        return;
    }
	detectController->robotWorker->loadTransformMatrix(file_name);
	cv::Mat T = detectController->robotWorker->getTransformMatrix();
	detectController->ppfWorker->camTend = detectController->robotWorker->getTransformMatrix().clone();
	//std::cout << "6Treals:" << T << std::endl;
	std::ostringstream oss;
	oss << "T = \n   [ " << T.at<double>(0, 0) << " " << T.at<double>(0, 1) << " " << T.at<double>(0, 2) << " " << T.at<double>(0, 3) << " ]"
		<< "\n   [ " << T.at<double>(1, 0) << " " << T.at<double>(1, 1) << " " << T.at<double>(1, 2) << " " << T.at<double>(1, 3) << " ]"
		<< "\n   [ " << T.at<double>(2, 0) << " " << T.at<double>(2, 1) << " " << T.at<double>(2, 2) << " " << T.at<double>(2, 3) << " ]"
		<< "\n   [ " << T.at<double>(3, 0) << " " << T.at<double>(3, 1) << " " << T.at<double>(3, 2) << " " << T.at<double>(3, 3) << " ]";
    ui->labelMatrixText->setText(QString::fromStdString(oss.str()));

	calibrationPrepared = true;
	if (modelPrepared && cameraPrepared && calibrationPrepared && objectDetected && movedToCamera0)
		{
			ui->pushButton_Pick1stObject->setEnabled(true);
			ui->pushButton_StartSequence->setEnabled(true);
		}

}

void
PCLViewer::startSimulationButtonPressed ()
{
	ui->pushButton_StartSimulation->setEnabled(false);
	ui->checkBoxVisualize->setEnabled(false);
	detectController->simulationWorker->setting.visualization = ui->checkBoxVisualize->isChecked();
	simulationViewer->setCameraPosition(0, 16, 93,      0, 16, 0,     0, 1, 0);
	emit detectController->operate("makeBinScene");
}

void
PCLViewer::simulationDetectObjectsButtonPressed()
{
	ui->pushButton_DetectObjects->setEnabled(false);
	ui->pushButton_SimulationDetectObjects->setEnabled(false);
	PointCloudType::Ptr simuCloud(new PointCloudType());
	pcl::copyPointCloud(*(detectController->simulationWorker->save_cloud), *simuCloud);
	detectController->ppfWorker->storeLatestCloud(simuCloud);
    emit detectController->operate("simulationDetection");
}

void
PCLViewer::connectToNachiSocketButtonPressed()
{
	if (detectController->robotWorker->soc.initializeSocket())
	{
		std::cout << "Connected ...!" << std::endl;
		ui->pushButton_ConnectToNachiSocket->setEnabled(false);
//		detectController->ppfWorker->soc = std::make_shared<NachiSocket>(detectController->robotWorker->soc);
	}
		

}

void
PCLViewer::startRealsenseCameraButtonPressed()
{
	//std::cout << "Initializing Realsense ... !" << std::endl;
	// -------------------------------------------- Realsense Data Retrieve ----------------------------------------------------
	std::function<void(const std::shared_ptr<const PointCloudType >& cloud)> f_cloud =
		[this](const PointCloudType::ConstPtr& cloud) {
		realsenseCloud = cloud->makeShared();
	};

	std::function<void(const std::shared_ptr<rs2::video_frame>& rgb)> f_image =
		[this](const std::shared_ptr<rs2::video_frame>& rgb) {
		cv::Mat rgbColor(cv::Size(640, 480), CV_8UC3, (void*)rgb->get_data(), cv::Mat::AUTO_STEP);
        realsenseImage = rgbColor.clone();
	};

	reals.registerCallbackFunction(f_cloud, f_image);

	if (!reals.run()) {
		std::cout << "Error loading Realsense! Exiting... " << std::endl;
		return;
	}
	//std::cout << "Done Initializing Camera ... !" << std::endl;

    ui->pushButton_StartRealsenseCamera->setEnabled(false);
    ui->pushButton_RemoveBackground->setEnabled(true);

    timerRealsense = new QTimer(this);
    connect(timerRealsense, SIGNAL(timeout()), this, SLOT(updateRealsense2D()));
    connect(timerRealsense, SIGNAL(timeout()), this, SLOT(updateRealsense3D()));
    timerRealsense->start(100);
}

void
PCLViewer::removeBackgroundButtonPressed()
{
    ui->pushButton_RemoveBackground->setEnabled(false);
    cameraPrepared = true;
    if (modelPrepared && cameraPrepared)
        ui->pushButton_DetectObjects->setEnabled(true);

	timerRemoveBackground = new QTimer(this);
    connect(timerRemoveBackground, SIGNAL(timeout()), this, SLOT(updateRemoveBackground()));
	timerRemoveBackground->start(500);
}

void
PCLViewer::detectObjectsButtonPressed()
{
	ui->pushButton_DetectObjects->setEnabled(false);
	ui->pushButton_SimulationDetectObjects->setEnabled(false);

	//Update latest cloud from Detect Controller
	detectController->ppfWorker->storeLatestCloud(realsenseCloud);

    //3D Matching from Detect Controller
    emit detectController->operate("detectObjects");
}

void
PCLViewer::moveToCamera0ButtonPressed()
{
    ui->pushButton_Pick1stObject->setEnabled(false);
    ui->pushButton_DetectObjects->setEnabled(false);
    ui->pushButton_MoveToCamera0->setEnabled(false);
    ui->pushButton_StartSequence->setEnabled(false);
    emit detectController->operate("moveToCamera0");
}

void
PCLViewer::pick1stObjectButtonPressed()
{
    ui->pushButton_Pick1stObject->setEnabled(false);
    ui->pushButton_DetectObjects->setEnabled(false);
    ui->pushButton_MoveToCamera0->setEnabled(false);
    ui->pushButton_StartSequence->setEnabled(false);
    emit detectController->operate("pickObjectAndPlace");
}

void
PCLViewer::startSequenceButtonPressed()
{
	connectToNachiSocketButtonPressed();
    ui->pushButton_Pick1stObject->setEnabled(false);
    ui->pushButton_DetectObjects->setEnabled(false);
    ui->pushButton_MoveToCamera0->setEnabled(false);
    ui->pushButton_StartSequence->setEnabled(false);
    emit detectController->operate("startSequence");
}

void
PCLViewer::updateRealsense2D()
{
	cv::Mat rgbColor = realsenseImage.clone();
    QImage QimageRealsense = QImage((uchar*) rgbColor.data, rgbColor.cols, rgbColor.rows, rgbColor.step, QImage::Format_RGB888);
    ui->RealsenseImageLabel->setPixmap(QPixmap::fromImage(QimageRealsense));
}

void
PCLViewer::updateRealsense3D()
{
    realsense3DHandler->setInputCloud(realsenseCloud);
    realsenseViewer->updatePointCloud (realsenseCloud, *realsense3DHandler, "Realsense 3D");
    ui->qvtkWidgetCamera3D->update ();
}

void
PCLViewer::updateRemoveBackground()
{
	remove_background(realsenseCloud, 0.005, removeBackgroundCloud);
	//std::vector<float> passthrough_lim = { 0, 1, -0.145, 0.145, -0.06, 0.06 };
	//passthrough(realsenseCloud, passthrough_lim, removeBackgroundCloud);

    removeBackgroundHandler->setInputCloud(removeBackgroundCloud);
    removeBackgroundViewer->updatePointCloud (removeBackgroundCloud, *removeBackgroundHandler, "Remove Background");
    ui->qvtkWidgetRemoveBackground->update ();
}

void
PCLViewer::updateUi(const QString & parameter)
{
	if (parameter == "modelViewer")
	{
		ui->pushButton_BrowseModel->setEnabled(true);
		model = detectController->ppfWorker->getModel()->makeShared();

		modelHandler->setInputCloud(model);
		modelViewer->updatePointCloud (model, *modelHandler, "model");
		ui->qvtkWidgetModel->update ();

		simulationCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		simulationHandler->setInputCloud(simulationCloud);
		simulationViewer->updatePointCloud (simulationCloud, *simulationHandler, "simulation");
		ui->qvtkWidgetSimulation->update ();

		//Update model from Detect Controller
		detectController->simulationWorker->loadModel(model);
		emit detectController->operate("prepareModelDescriptor");
		//Reset
		ui->pushButton_StartSimulation->setEnabled(true);
		ui->checkBoxVisualize->setEnabled(true);

		modelPrepared = true;
		if (modelPrepared && cameraPrepared)
			ui->pushButton_DetectObjects->setEnabled(true);
		return;
	}
	if (parameter == "simulationViewer")
	{
		ui->qvtkWidgetSimulation->update ();
		return;
	}
	if (parameter == "detectionViewer")
	{
		ui->qvtkWidgetObjects->update ();
		return;
	}
	if (parameter == "enableDetectionButton")
	{
		if (!ui->pushButton_RemoveBackground->isEnabled() && !ui->pushButton_StartRealsenseCamera->isEnabled())
		{
			ui->pushButton_DetectObjects->setEnabled(true);
			objectDetected = true;
			if (modelPrepared && cameraPrepared && calibrationPrepared && objectDetected && movedToCamera0)
			{
				ui->pushButton_Pick1stObject->setEnabled(true);
				ui->pushButton_StartSequence->setEnabled(true);
			}
				
		}
		
		if (!ui->pushButton_StartSimulation->isEnabled())
			ui->pushButton_SimulationDetectObjects->setEnabled(true);
		return;
	}
	if (parameter == "moveToCamera0Done")
	{
        ui->pushButton_MoveToCamera0->setEnabled(true);
		if (!ui->pushButton_RemoveBackground->isEnabled() && !ui->pushButton_StartRealsenseCamera->isEnabled())
			ui->pushButton_DetectObjects->setEnabled(true);

		movedToCamera0 = true;
		if (modelPrepared && cameraPrepared && calibrationPrepared && objectDetected && movedToCamera0)
		{
			ui->pushButton_Pick1stObject->setEnabled(true);
			ui->pushButton_StartSequence->setEnabled(true);
		}
		return;
	}
    if (parameter == "robotCommunicationDone")
	{
        ui->pushButton_Pick1stObject->setEnabled(true);
        ui->pushButton_DetectObjects->setEnabled(true);
        ui->pushButton_MoveToCamera0->setEnabled(true);
        ui->pushButton_StartSequence->setEnabled(true);
		return;
	}
	
}

void PCLViewer::doWork(const MyArray& pos)
{
	//Update latest cloud from Detect Controller
	detectController->ppfWorker->storeLatestCloud(realsenseCloud);
	emit detectController->operateDetection(pos);
}

PCLViewer::~PCLViewer ()
{
  detectController->robotWorker->soc.closeSocket();
  delete ui;
}
