// Khai bao, dinh nghia cac ham Signals va Slots
#pragma once

#include <iostream>

// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>

// Point Cloud Library
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

//VTK
#include <vtkWin32OutputWindow.h>

//OpenCV
#include "opencv2/opencv.hpp"

//User Library
#include "include/meshSampling/meshSampling.h"
#include "include/dataType/dataType.h"
#include "include/HPR/HPR.h"
#include "include/binSceneMaker/binSceneMaker.h"
#include "include/realsense/realsense.h"
#include "include/pclFunction/pclFunction.h"
#include "include/robotCommunication/robotCommunication.h"
#include "DetectionWorker.h"

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

namespace Ui
{
    class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLViewer (QWidget *parent = 0);
    ~PCLViewer ();

public Q_SLOTS:
    void
    browseModelButtonPressed ();

    void
    browseTransformMatrixButtonPressed();
 
    void
    startSimulationButtonPressed();

    void
    simulationDetectObjectsButtonPressed();

    void
    connectToNachiSocketButtonPressed();

    void
    startRealsenseCameraButtonPressed();

    void
    removeBackgroundButtonPressed();

    void
    detectObjectsButtonPressed();

    void
    moveToCamera0ButtonPressed();

    void
    pick1stObjectButtonPressed();

    void
    startSequenceButtonPressed();

    void
    updateRealsense2D();

    void
    updateRealsense3D();

    void
    updateRemoveBackground();

	void
    updateUi(const QString & parameter);

	void doWork(const MyArray& pos);

protected:
    //Model
    pcl::PointCloud<pcl::PointXYZ>::Ptr model;

    pcl::visualization::PCLVisualizer::Ptr modelViewer;

    //Simulation
    pcl::visualization::PCLVisualizer::Ptr simulationViewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr simulationCloud;

    //Realsense camera
    QTimer * timerRealsense;
    Realsense reals;
    pcl::PointCloud<PointType>::Ptr realsenseCloud;
    cv::Mat realsenseImage;
    pcl::visualization::PCLVisualizer::Ptr realsenseViewer;

    //Remove background
	QTimer * timerRemoveBackground;
    pcl::PointCloud<PointType>::Ptr removeBackgroundCloud;
    pcl::visualization::PCLVisualizer::Ptr removeBackgroundViewer;

	//Detect Objects
	DetectionController* detectController;
    pcl::PointCloud<PointType>::Ptr detectObjectsCloud;
    pcl::visualization::PCLVisualizer::Ptr detectObjectsViewer;

private:
    bool modelPrepared = false;
	bool calibrationPrepared = false;
    bool cameraPrepared = false;
	bool objectDetected = false;
	bool movedToCamera0 = false;
    Ui::PCLViewer *ui;


    std::shared_ptr<pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>> modelHandler =
        std::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>>(0, 0, 255);

    std::shared_ptr<pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>> simulationHandler =
        std::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>>(255, 255, 255);

    std::shared_ptr<pcl::visualization::PointCloudColorHandler<PointType>> realsense3DHandler =
        std::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<PointType>>();

    std::shared_ptr<pcl::visualization::PointCloudColorHandler<PointType>> removeBackgroundHandler =
        std::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<PointType>>();
	
    std::shared_ptr<pcl::visualization::PointCloudColorHandler<PointType>> detectObjectsHandler =
        std::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<PointType>>();

	//std::shared_ptr<pcl::visualization::PointCloudColorHandler<PointType>> realsense3DHandler =
 //       std::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>>(255, 255, 255);

 //   std::shared_ptr<pcl::visualization::PointCloudColorHandler<PointType>> removeBackgroundHandler =
 //       std::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>>(255, 255, 255);

 //   std::shared_ptr<pcl::visualization::PointCloudColorHandler<PointType>> detectObjectsHandler =
 //       std::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>>(255, 255, 255);




};
