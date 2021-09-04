#ifndef BINSCENEMAKER
#define BINSCENEMAKER

#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable:4819)
#pragma once

#include <iostream>
#include <thread>
#include <random>
#include <memory>
#include <chrono>
#include <stdexcept>
#include <cstdlib>

#include <omp.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/geometry.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btIDebugDraw.h>

#include <QObject>
#include <QThread>
#include "convexDecomposition.hpp"

#include "HPR.h"

template<typename T> constexpr const T pi() { return static_cast<T>(std::atan2(0.0, -1.0)); }

class BinSceneMakerSetting
{
public:
	int model_num;
	double precapture_wait;
	int falling_interval;
	int postsimulation_wait;
	int random_seed;
	double cup_r;
	double cup_h;
	double cup_restitution;
	double cup_friction;
	double model_restitution;
	double model_friction;
	double normalize_length;
	int downsample_target_points_num;
	double downsample_initial_leaf_size;
	double downsample_factor;
	bool visualization;
	bool capture_screenshots;

	BinSceneMakerSetting():
		model_num(12),
		precapture_wait(0),
		falling_interval(100),
		postsimulation_wait(1000),
		random_seed(time(0)),
		cup_r(1.2),
		cup_h(3.0),
		cup_restitution(0.05),
		cup_friction(0.3),
		model_restitution(0.05),
		model_friction(0.3),
		normalize_length(10.0),
		downsample_target_points_num(2000),
		downsample_initial_leaf_size(0.001),
		downsample_factor(1.2),
		visualization(false),
		capture_screenshots(false)
	{}
};

class BinSceneMaker : public QObject
{
	Q_OBJECT

public Q_SLOTS:
	void doWork(const QString &parameter);

Q_SIGNALS:
	void resultReady(const QString &result);
	void updateUiFromWorker(const QString &);

public:
	BinSceneMakerSetting setting;

	pcl::PointCloud<pcl::PointXYZ>::Ptr model;
	pcl::visualization::PCLVisualizer::Ptr simulationViewer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr save_cloud;

	BinSceneMaker();
	void loadModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& model_);
	bool makeBinScene();


	

};

#endif