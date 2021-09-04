#pragma once
//Qt Library
#include <pcl/console/time.h>
#include <fstream>

#include <QObject>
#include <QThread>
#include <QVector>

#include "dataType.h"
#include "NachiSocket.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>

namespace Eigen {
	template<class Matrix>
	void write_binary(const char* filename, const Matrix& matrix) {
		std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
		typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
		out.write((char*)(&rows), sizeof(typename Matrix::Index));
		out.write((char*)(&cols), sizeof(typename Matrix::Index));
		out.write((char*)matrix.data(), rows*cols * sizeof(typename Matrix::Scalar));
		out.close();
	}
	template<class Matrix>
	void read_binary(const char* filename, Matrix& matrix) {
		std::ifstream in(filename, std::ios::in | std::ios::binary);
		typename Matrix::Index rows = 0, cols = 0;
		in.read((char*)(&rows), sizeof(typename Matrix::Index));
		in.read((char*)(&cols), sizeof(typename Matrix::Index));
		matrix.resize(rows, cols);
		in.read((char *)matrix.data(), rows*cols * sizeof(typename Matrix::Scalar));
		in.close();
	}
} // Eigen::
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> Matrix_MxN;


class RobotCommunication : public QObject
{
	Q_OBJECT

public Q_SLOTS:
	void doWork(const QString &parameter);

Q_SIGNALS:
	void operateDetection(const MyArray& pos);
	void resultReady(const QString &result);

private:
	bool object_available = false;
	cv::Mat camTend;
	Eigen::Matrix4d eigenCamTend;
	std::mutex mtx;
	int numberOfShots = 0;
	int objectsPicked = 0;   // De tha
	pcl::console::TicToc tt; // Tictoc for process-time calculation

	float roll1, pitch1, yaw1;
	//float preCamera0Position[6] = { 150, -425, 295, -90, 0 , -180 };
	//float camera0Position[6] = { 170, -425, 295, -90, 0 , -180 }; 
	//float camera0Position[6] = { 42.69, -480.63, 501.67, 85.8, -1.6 , -180 };

	float prePlace1[6] = { -219.76, -331.26, 60, 0.06, -0.02, -179.95 };
	float prePlace2[6] = { -219.76, -431.26, 60, 0.06, -0.02, -179.95 };
	float prePlace3[6] = { -219.76, -531.26, 60, 0.06, -0.02, -179.95 };
	float prePlace4[6] = { -119.76, -331.26, 60, 0.06, -0.02, -179.95 };
	float prePlace5[6] = { -119.76, -431.26, 60, 0.06, -0.02, -179.95 };
	float prePlace6[6] = { -119.76, -531.26, 60, 0.06, -0.02, -179.95 };

	/*float Place1[6] = { -219.76, -331.26, 7.47, 0.06, -0.02, -179.95 };
	float Place2[6] = { -235, -548.02, 7.6, 1.82, 0.16, 176.84 };*/

	float Place1[6] = { -219.76, -331.26, 7.47, 0.06, -0.02, -179.95 };
	float Place2[6] = { -219.76, -431.26, 7.47, 0.06, -0.02, -179.95 };
	float Place3[6] = { -219.76, -531.26, 7.47, 0.06, -0.02, -179.95 };
	float Place4[6] = { -119.76, -331.26, 7.47, 0.06, -0.02, -179.95 };
	float Place5[6] = { -119.76, -431.26, 7.47, 0.06, -0.02, -179.95 };
	float Place6[6] = { -119.76, -531.26, 7.47, 0.06, -0.02, -179.95 };


	float preCamera0Position[6] = { 82.13, -489.66, 234.74, 1.98, -0.31 , 180 };
	float preprecamera0Position[6] = { 72.13, -489.66, 234.74, 1.98, -0.31 , 180 };
	float camera0Joint[6] = { -80.58,68.99,-5.64,0.26,-62.78,-82.68 };  // Camera 0 ,doi tu XYZ-> 6 goc


	/////////////////////////////////////Manual.......

	float prePick1[6] = { -72.64, 65.62, -36.45, -16.31, -15.8 , -59.29 }; ////// Gia tri Joint - Angle
	float prePick2[6] = { -77.05, 50.37, -9.61, -0.35, -38.45, -78.77 };

	float Pick1[6] = { 120.89, -465.97, 14.08, 1.77, 0.53 , 165.39 };		////Gia tri LIN XYZ RPY
	float Pick2[6] = { 120.96, -518.98, 6.68, 1.99, -0.23 , 177.7 };

	float aftPick1[6] = { -75.19, 62.38, -20.51, -1.39, -38.84, -75.95 };

	

	
	//////////////////////////////////////////

	std::vector<Eigen::Matrix4d> objectTransfromMatFromCam;
	Eigen::Matrix4d objectTransfromMatFromBase;
	Eigen::Matrix4d objectTransfromMatFromCamChosen;
	std::vector<double> rpw;

	std::vector<double> alpha{ M_PI / 2, 0, M_PI / 2, -M_PI / 2, M_PI / 2, 0 };
	std::vector<double> a{ 0.05, 0.33, 0.045, 0, 0, 0 };
	//Chu y d6
	std::vector<double> d{ 0.345, 0, 0, 0.340, 0, 0.083 };
	float theta[6] = { -80.63, 68.53, -5, 0.34, -63.5, -82.76 };
public:
	NachiSocket soc;


	RobotCommunication();
	cv::Mat getTransformMatrix();
	void loadTransformMatrix(QString file_name);
	void loadObjectTransfromMatFromCam(const std::vector<Eigen::Matrix4d>& objectTransfromMatFromCam_);
	void calculatePositionFromRobot();
	void moveToCamera0();
	void moveToDetectionPosition();
	//void moveToDetectionPositionUpRight();
	bool pickObjectAndPlace();
	void startSequence();

};
