#include "robotCommunication.h"

void getTransMatrixJointi(double alpha, double a, double d, float theta, cv::Mat& transformMatrix)
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

RobotCommunication::RobotCommunication() {}

void RobotCommunication::doWork(const QString &parameter)
{
	QString result;
	if (parameter == "moveToCamera0")
	{
		moveToCamera0();
		result = "moveToCamera0Success";
		emit resultReady(result); // Phat tin hieu thuc hien xong lenh move
		return;
	}
	if (parameter == "pickObjectAndPlace")
	{
		calculatePositionFromRobot();
		pickObjectAndPlace();
		result = "pickObjectAndPlaceSuccess";
		emit resultReady(result);
		return;
	}
	if (parameter == "startSequence")
	{
		std::cout << "Step 1: Move to READY position." << std::endl;
		moveToCamera0();
		MyArray cam0 = { camera0Joint[0], camera0Joint[1], camera0Joint[2], camera0Joint[3], camera0Joint[4], camera0Joint[5] };
		emit operateDetection(cam0);   //Phat tin hieu de bat dau Detect vat
		std::cout << "Step 2: Detect objects' poses." << std::endl;
		return;
	}
	if (parameter == "pickObjectAndPlaceSequence")
	{
		calculatePositionFromRobot();
		/*if (abs(rpw[0]) > 50 || abs(rpw[1]) < 130 || objectTransfromMatFromBase(2,3) < 0.001)
		{
			std::cout << "Detection FAILED. Redetecting..." << std::endl;
			MyArray cam0 = { camera0Joint[0], camera0Joint[1], camera0Joint[2], camera0Joint[3], camera0Joint[4], camera0Joint[5] };			
			emit operateDetection(cam0);
			return;
		}*/

		///// Kiem tra lai dieu kien khi dat nghieng
		//if ((objectTransfromMatFromCamChosen(0, 2) < -0.97) && (objectTransfromMatFromCamChosen(1, 1) < -0.97) && (objectTransfromMatFromCamChosen(2, 0) < -0.97))
		if (abs(objectTransfromMatFromCamChosen(2, 0)) > 0.96)
		{
			std::cout << "Mode Test: " << std::endl;
			std::cout << "\nobjectsPicked: \t" << objectsPicked << std::endl;
			//moveToDetectionPosition();
			Eigen::MatrixXd detectPositionTEST(4, 1);
			detectPositionTEST << 0, 0, 0, 1;
			std::cout << "OTBase" << objectTransfromMatFromBase << std::endl;
			detectPositionTEST = objectTransfromMatFromBase * detectPositionTEST;
			//std::cout << "xyz rpw Dectect Position so voi Base: \t" << (detectPositionTEST(0, 0) * 1000) << "\t" << (detectPositionTEST(1, 0) * 1000) << (detectPositionTEST(2, 0) * 1000) << "\t" << 0.0f << "\t" << 0.0f << "\t" << -180.0f << std::endl;
			/*float detectDestinationTEST[6] = {(detectPositionTEST(0,0) * 1000), (detectPositionTEST(1,0) * 1000), (detectPositionTEST(2,0) * 1000),
										0.0, 0.0, -180.0 };*/
			float detectDestinationTEST[6] = { (detectPositionTEST(0,0) * 1000), (detectPositionTEST(1,0) * 1000), (detectPositionTEST(2,0) * 1000),
										roll1, pitch1, yaw1 };

			soc.moveRobotTo(detectDestinationTEST, ABS_LINEAR);
			soc.moveRobotTo(detectDestinationTEST, COMPRESSED_AIR_9);
			soc.moveRobotTo(aftPick1, ABS_JOINT);
			std::cout << "- Move to Placing position " << std::endl;

			//prepPlace[2] = 50.0;
			if (objectsPicked == 0)
			{
				soc.moveRobotTo(prePlace1, ABS_LINEAR_150);
				soc.moveRobotTo(Place1, ABS_LINEAR);
				soc.moveRobotTo(Place1, COMPRESSED_AIR_10);
				soc.moveRobotTo(prePlace1, ABS_LINEAR_150);
			}
			else if (objectsPicked == 1)
			{
				soc.moveRobotTo(prePlace2, ABS_LINEAR_150);
				soc.moveRobotTo(Place2, ABS_LINEAR);
				soc.moveRobotTo(Place2, COMPRESSED_AIR_10);
				soc.moveRobotTo(prePlace2, ABS_LINEAR_150);
			}
			else if (objectsPicked == 2)
			{
				soc.moveRobotTo(prePlace3, ABS_LINEAR_150);
				soc.moveRobotTo(Place3, ABS_LINEAR);
				soc.moveRobotTo(Place3, COMPRESSED_AIR_10);
				soc.moveRobotTo(prePlace3, ABS_LINEAR_150);

			}
			else if (objectsPicked == 3)
			{
				soc.moveRobotTo(prePlace4, ABS_LINEAR_150);
				soc.moveRobotTo(Place4, ABS_LINEAR);
				soc.moveRobotTo(Place4, COMPRESSED_AIR_10);
				soc.moveRobotTo(prePlace4, ABS_LINEAR_150);

			}
			else if (objectsPicked == 4)
			{
				soc.moveRobotTo(prePlace5, ABS_LINEAR_150);
				soc.moveRobotTo(Place5, ABS_LINEAR);
				soc.moveRobotTo(Place5, COMPRESSED_AIR_10);
				soc.moveRobotTo(prePlace5, ABS_LINEAR_150);

			}
			else if (objectsPicked == 5)
			{
				soc.moveRobotTo(prePlace6, ABS_LINEAR_150);
				soc.moveRobotTo(Place6, ABS_LINEAR);
				soc.moveRobotTo(Place6, COMPRESSED_AIR_10);
				soc.moveRobotTo(prePlace6, ABS_LINEAR_150);
			}
			
			objectsPicked++;
			std::cout << "\nObjectsPicked: \t" << objectsPicked << std::endl;
			soc.moveRobotTo(preprecamera0Position, ABS_LINEAR_150);
			soc.moveRobotTo(preCamera0Position, ABS_LINEAR);
			//std::this_thread::sleep_for(std::chrono::microseconds(500));
			
		}
		else
		//if ((objectTransfromMatFromCamChosen[0,2] > -0.98) || (objectTransfromMatFromCamChosen[1, 1] > -0.98) || (objectTransfromMatFromCamChosen[2, 0] > -0.98))
		/*if ((rpw[0]) < 96 ||(rpw[0]) > 105 || (rpw[1]) < -108 || (rpw[1]) > -103)*/
		{
			std::cout << "Detection FAILED. Redetecting..." << std::endl;
			MyArray cam0 = { camera0Joint[0], camera0Joint[1], camera0Joint[2], camera0Joint[3], camera0Joint[4], camera0Joint[5] };
			emit operateDetection(cam0);
			return;
		}
		
		//if ((abs(objectTransfromMatFromCamChosen(2,3)) > 0.3 //z
		//	|| abs(objectTransfromMatFromCamChosen(0,3)) > 0.03 //x
		//	|| abs(objectTransfromMatFromCamChosen(1,3)) > 0.03 //y
		//	|| abs(rpw[0]) > 30 || abs(rpw[1]) < 150) 
		//	 && numberOfShots < 2
		//	)
		//	{
		//		std::cout << "Detection not clear. Moving to better angle..." << std::endl;
		//		std::cout << numberOfShots << std::endl;
		//		if (numberOfShots == 0)
		//			{
		//				moveToDetectionPositionUpRight();
		//				std::cout << "UR" << std::endl;
		//				numberOfShots++;
		//			}
		//		else
		//			{
		//				moveToDetectionPosition();
		//				std::cout << "1" << std::endl;
		//				numberOfShots++;
		//			}
		//		return;
		//	}
		//numberOfShots = 0;

		tt.tic();
		std::cout << "Step 3: Pick and Place sequence:" << std::endl;
		if (!pickObjectAndPlace())
		{
			std::cout << "ALL OBJECTS ARE PICKED!!!" << std::endl;
			result = "startSequenceSuccess";
			emit resultReady(result); // Phat tin hieu la da thuc hienj xong tac vu Pick and Place
			return;
		}
		std::cout << "Grip in " << tt.toc() << " mseconds..." << std::endl;

		std::cout << "REPEAT the whole process." << std::endl << std::endl;
		std::cout << "Step 1: Move to READY position." << std::endl;
		MyArray cam0 = { camera0Joint[0], camera0Joint[1], camera0Joint[2], camera0Joint[3], camera0Joint[4], camera0Joint[5] };
		emit operateDetection(cam0);
		std::cout << "Step 2: Detect objects' poses." << std::endl;
		return;
	}
}

/**
* @brief loadTransformMatrix loads the transformation matrix from Camera coordinate
*	       to End Effector coordinate
* @param file_name - input .dat file
* @return void
*/
void RobotCommunication::loadTransformMatrix(QString file_name)
{
	Matrix_MxN EigenInputMatrix;
	Eigen::read_binary(file_name.toStdString().c_str(), EigenInputMatrix);
	cv::Mat _6Treals;
	cv::eigen2cv(EigenInputMatrix, _6Treals);
	_6Treals.convertTo(_6Treals, 6);
	camTend = _6Treals.inv();
	cv::cv2eigen(camTend, eigenCamTend);
	/*std::cout << "\n1" << std::endl;*/
}

cv::Mat  RobotCommunication::getTransformMatrix()
{
	return camTend;
}

void RobotCommunication::loadObjectTransfromMatFromCam(const std::vector<Eigen::Matrix4d>& objectTransfromMatFromCam_)
{
	mtx.lock();
	objectTransfromMatFromCam = objectTransfromMatFromCam_;
	mtx.unlock();
}


void RobotCommunication::calculatePositionFromRobot()
{
	//soc.getJointPosition(theta);
	//soc.getEndEffectorPosition(theta);

	cv::Mat T60 = cv::Mat::eye(cv::Size(4, 4), 6);
	for (int i = 0; i < 6; ++i)
	{
		cv::Mat Tjointi;
		getTransMatrixJointi(alpha[i], a[i], d[i], theta[i] * M_PI / 180, Tjointi);
		T60 *= Tjointi;
	}

	//std::cout << "\n Matrix 0T6 " << T60 << std::endl;

	cv::Mat cvT(4,4,CV_32FC1); 

	Eigen::Matrix4d eigenT60;
	cv::cv2eigen(T60, eigenT60);

	//std::cout << "\nObjTcam.size: \t" << objectTransfromMatFromCam.size() << std::endl;	

	if (objectTransfromMatFromCam.size() > 0)
	{
		object_available = true;
		mtx.lock();
		objectTransfromMatFromCamChosen = objectTransfromMatFromCam[0];
		mtx.unlock();

		//Tim vat nam tren cung
		for (int i = 0; i < objectTransfromMatFromCam.size(); ++i)
		{
			if (objectTransfromMatFromCam[i](2, 3) < objectTransfromMatFromCamChosen(2, 3))
			{
				objectTransfromMatFromCamChosen = objectTransfromMatFromCam[i];
			}
			
		}
		//std::cout << " Matrix objectTCamChosen " << objectTransfromMatFromCamChosen << std::endl;
		//Dieu chinh
		Eigen::Matrix4d FineTuningMatRot;
		FineTuningMatRot << cos(-2.5*M_PI/180), -sin(-2.5*M_PI/180), 0, 0,
			sin(-2.5*M_PI/180), cos(-2.5*M_PI/180), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		Eigen::Matrix4d FineTuningMatTrans;
		FineTuningMatTrans << 1, 0, 0, 0.008,
			0, 1, 0, -0.008,
			0, 0, 1, 0.005,
			0, 0, 0, 1;

		//objectTransfromMatFromBase = eigenT60 * eigenCamTend * objectTransfromMatFromCamChosen * FineTuningMatRot * FineTuningMatTrans;
		objectTransfromMatFromBase = eigenT60 * eigenCamTend * objectTransfromMatFromCamChosen;
		//std::cout << "objectTransfromMatFromBase: " << objectTransfromMatFromBase << std::endl;

		objectTransfromMatFromBase(0, 3) += -0.042-0.002;
		objectTransfromMatFromBase(1, 3) += -0.009;
		objectTransfromMatFromBase(2, 3) += 0.036;
		std::cout << "objectTransfromMatFromBase: " << objectTransfromMatFromBase << std::endl;


		//Chuyen tu ma tran xoay thanh 3 goc Roll Pitch Yaw
		Eigen::Matrix3d rotation;
		rotation << objectTransfromMatFromBase(0, 0), objectTransfromMatFromBase(0, 1), objectTransfromMatFromBase(0, 2),
					objectTransfromMatFromBase(1, 0), objectTransfromMatFromBase(1, 1), objectTransfromMatFromBase(1, 2),
					objectTransfromMatFromBase(2, 0), objectTransfromMatFromBase(2, 1), objectTransfromMatFromBase(2, 2);
		
		//std::cout << "\n Rotation Matrix: \n" << rotation << std::endl;
		
		yaw1 = std::atan2(rotation(1, 0), rotation(0, 0));
		pitch1 = std::atan2(-rotation(2, 0), std::sqrt(rotation(0, 0)*rotation(0, 0) + rotation(1, 0)*rotation(1, 0)));
		roll1 = std::atan2(rotation(2, 1), rotation(2, 2));
		roll1 *= 180 / M_PI;
		pitch1 *= 180 / M_PI;
		yaw1 *= 180 / M_PI;
		/*std::cout << "Method1: Atan2 Roll Pitch Yaw angles:" << std::endl;
		std::cout << "Roll = " << roll1 << std::endl << "Pitch = " << pitch1 << std::endl << "Yaw = " << yaw1 << std::endl << std::endl << std::endl;*/
		if ((roll1 <= -74 && roll1 >= -90) ) roll1 = 0;
	
		if ((pitch1 <= -74 && pitch1 >= -90)) pitch1 = 0;
		yaw1 = -179.99;
		
		if (objectTransfromMatFromBase(2, 3) > 0.012)
		{
			roll1 = 1.95;
			pitch1 = -0.14;
			yaw1 = 162.10;
		}
		else
		{
			roll1 = 0;
			pitch1 = 0;
			yaw1 = -179.99;
		}
		//////////////////////////////////////////////////
		//Eigen::Vector3d ea = rotation.eulerAngles(0, 1, 2);  // Don vi radian
		//
		//// Tuc la tuong ung voi ma tran xoay tu Obj ve khau 0, ta se quay theo thu tu (tinh he toan do Obj) x: ea(0) do; quay quanh y ea(1) do; quay quanh z: ea(2)
		//rpw.clear(); // Xoa mang de luu mang rpw theo Euler
		//std::cout << "raw Euler angles:" << std::endl;
		//std::cout << ea(0) << std::endl << ea(1) << std::endl << ea(2) << std::endl << std::endl << std::endl;
		//for (int i = 0; i < ea.size(); ++i)
		//{
		//	rpw.push_back(ea(i)); // Hien tai rpw dang la rad. ea(1) goc qua quanh truc x; ea(2): goc quay quanh truc y; ea(3) : goc quay quanh z
		//	rpw[i] *= 180 / M_PI; //Doi sang do
		//	if (rpw[i] > 180)
		//		rpw[i] -= 360;
		//	else if (rpw[i] < -180)
		//		rpw[i] += 360;
		//} // Dua cac goc ve range tinh toan tu (-180;180)
		/*std::cout << rpw[0] << std::endl << rpw[1] << std::endl << rpw[2] << std::endl << std::endl << std::endl;

		if (rpw[0] > 90 && rpw[0] < 180)
			rpw[0] -= 180;
		if (rpw[1] < 90 && rpw[1] > 0)
			rpw[1] -= 180;
		else if (rpw[1] > 90 && rpw[1] < 180)
			rpw[1] = -rpw[1];
		else if (rpw[1] < 0 && rpw[1] > -90)
			rpw[1] += 180;
		else if (rpw[1] < -90 && rpw[1] > -180)
			rpw[1] += 0;
		rpw[2] = -89.99;*/

		//Kiem tra bang goc Euler angle ?
		/*std::cout << "processed Euler angles after:" << std::endl;
		std::cout << rpw[0] << std::endl << rpw[1] << std::endl << rpw[2] << std::endl << std::endl << std::endl;*/
		///////////////////////////////////////////////////////////
	}
	else
	{
		std::cout << "No more object " << std::endl;
		object_available = false;
	}

	
}

void RobotCommunication::moveToCamera0()
{
	soc.moveRobotTo(preCamera0Position, ABS_LINEAR_150);
	//soc.moveRobotTo(camera0Position, ABS_LINEAR);
	//soc.moveRobotTo(preCamera0Position, COMPRESSED_AIR_10);
	//std::this_thread::sleep_for(std::chrono::microseconds(500));

}

void RobotCommunication::moveToDetectionPosition()
{
	///////////////////////
	Eigen::MatrixXd detectPosition(4, 1);
	detectPosition << 0, 0.05, -0.1, 1;
	detectPosition = objectTransfromMatFromBase * detectPosition;
	float detectDestination[6] = { (detectPosition(0,0) * 1000), (detectPosition(1,0) * 1000),(detectPosition(2,0) * 1000),roll1 , pitch1, yaw1 };
	//float detectDestination[6] = { (detectPosition(0,0) * 1000), (detectPosition(1,0) * 1000),(detectPosition(2,0) * 1000), 0.0f, 0.0f, -180.0f };
	if ((detectDestination[0, 2]  < 2)) detectDestination[0, 2] == 2;
	soc.moveRobotTo(detectDestination, ABS_LINEAR);
	std::this_thread::sleep_for(std::chrono::microseconds(500));

	//////////////////////Warning //////////////////////
	////////////////////////////
	float joint[6];
	soc.getJointPosition(joint); //Dang co loi khi doc gia tri Joint ve
	MyArray detPos = { joint[0], joint[1], joint[2], joint[3], joint[4], joint[5] };
	emit operateDetection(detPos);
/* Luat lam

	Eigen::MatrixXd detectPosition(4, 1);
	detectPosition << 0, 0, 0, 1;
	detectPosition = objectTransfromMatFromBase * detectPosition;
	std::cout << "OTBase" << objectTransfromMatFromBase << std::endl;
	std::cout << "xyz rpy Dectect Position so voi Base: \t" << detectPosition((0, 0) * 1000) << "\t" << detectPosition((1, 0) * 1000) << detectPosition((2, 0) * 1000) << "Roll = " << roll1 << std::endl << "Pitch = " << pitch1 << std::endl << "Yaw = " << yaw1 << std::endl;
	float detectDestination[6] = { detectPosition(0,0) * 1000, detectPosition(1,0) * 1000, detectPosition(2,0) * 1000, roll1 , pitch1, yaw1 };
	std::cout << "- Move to Picking position " << std::endl;
	if (detectDestination[0, 2] < 0.002) detectDestination[0, 2] == 0.002;
	std::cout << "z = " << detectDestination[0, 2] <<"mm"<< std::endl;
	soc.moveRobotTo(detectDestination, ABS_LINEAR_150);
	soc.moveRobotTo(detectDestination, COMPRESSED_AIR_9);
	soc.moveRobotTo(aftPick1, ABS_JOINT);
	soc.moveRobotTo(prePlace1, ABS_LINEAR_150);
	soc.moveRobotTo(Place1, ABS_LINEAR_150);
	soc.moveRobotTo(Place1, COMPRESSED_AIR_10);
	soc.moveRobotTo(prePlace1, ABS_LINEAR_150);
	soc.moveRobotTo(preCamera0Position, ABS_LINEAR_150);
	std::this_thread::sleep_for(std::chrono::microseconds(500));*/
	
	/////////////////////////

/*
	Eigen::MatrixXd detectPosition(4, 1);
	detectPosition << 0, 0.05, -0.1, 1;
	detectPosition = objectTransfromMatFromBase * detectPosition;
	std::cout << "xyz Dectect Position so voi Base: \t" << detectPosition((0, 0) * 1000) << "\t" << detectPosition((1, 0) * 1000) << detectPosition((2, 0) * 1000) << std::endl;
	float detectDestination[6] = { static_cast<float>(detectPosition(0,0) * 1000), static_cast<float>(detectPosition(1,0) * 1000), 
								static_cast<float>(detectPosition(2,0) * 1000),
								static_cast<float>(rpw[2]),  static_cast<float>(rpw[0]),  static_cast<float>(rpw[1]) };
	soc.moveRobotTo(detectDestination, ABS_LINEAR);
	std::this_thread::sleep_for(std::chrono::microseconds(500));*/

}
//
//void RobotCommunication::moveToDetectionPositionUpRight()
//{
//	Eigen::MatrixXd detectPosition(4, 1);
//	detectPosition << 0, 0, 0, 1;
//	detectPosition = objectTransfromMatFromBase * detectPosition;
//	detectPosition(0, 0) -= 0.05;
//	detectPosition(2, 0) += 0.1;
//	/*float detectDestination[6] = { static_cast<float>(detectPosition(0,0) * 1000), static_cast<float>(detectPosition(1,0) * 1000), 
//								static_cast<float>(detectPosition(2,0) * 1000),
//								-90.0f, 0.0f, 180.0f };*/
//	float detectDestination[6] = { static_cast<float>(detectPosition(0,0) * 1000), static_cast<float>(detectPosition(1,0) * 1000),
//								static_cast<float>(detectPosition(2,0) * 1000),
//								static_cast<float>(rpw[2]),  static_cast<float>(rpw[0]),  static_cast<float>(rpw[1]) };
//	soc.moveRobotTo(detectDestination, ABS_LINEAR);
//	std::this_thread::sleep_for(std::chrono::microseconds(500));
//	float joint[6];
//	soc.getJointPosition(joint);
//	MyArray detPos = { joint[0], joint[1], joint[2], joint[3], joint[4], joint[5] };
//	emit operateDetection(detPos);
//}

bool RobotCommunication::pickObjectAndPlace()
{
	//
	if (object_available)
	//{
	//	Eigen::MatrixXd prepPosition(4, 1);
	//	prepPosition << 0, 0, -0.05, 1;
	//	std::cout << "- ObjectT0: \n" << objectTransfromMatFromBase<< std::endl;
	//	prepPosition = objectTransfromMatFromBase * prepPosition;
	//	float prepDestination[6] = { (prepPosition(0,0) * 1000), (prepPosition(1,0) * 1000),(prepPosition(2,0) * 1000), 0.0f, 0.0f, -180.0f };
	//	std::cout << "X = " << prepDestination[0]<< "\n"
	//			  << "Y = " << prepDestination[1] << "\n"
	//			  <<" Z = " << prepDestination[2] <<"\n"
	//			  <<" Roll = " << prepDestination[3] << "\n"
	//		      <<" Pitch = " << prepDestination[4] << "\n"
	//		      <<" Yaw = " << prepDestination[5] << std::endl;
	//	std::cout << "- Move to gripping position " << std::endl; 
	//	if ((prepDestination[0, 2] < 2)) prepDestination[0, 2] == 2;
	//	soc.moveRobotTo(prepDestination, ABS_LINEAR);

	//	Eigen::MatrixXd prepgripPosition(4, 1);
	//	prepgripPosition << 0, 0, 0, 1;
	//	prepgripPosition = objectTransfromMatFromBase * prepgripPosition;
	//	float prepgripDestination[6] = { (prepgripPosition(0,0) * 1000),(prepgripPosition(1,0) * 1000), (prepgripPosition(2,0) * 1000), 0.0f, 0.0f, -180.0f };
	//	std::cout << "- Move to gripping position" << std::endl;
	//	if ((prepgripDestination[0, 2] < 2)) prepgripDestination[0, 2] == 2;
	//	soc.moveRobotTo(prepgripDestination, ABS_LINEAR);

	//	Eigen::MatrixXd gripPosition(4, 1);
	//	gripPosition << 0, 0, 0.002, 1;
	//	gripPosition = objectTransfromMatFromBase * gripPosition;
	//	float gripDestination[6] = {(gripPosition(0,0) * 1000),(gripPosition(1,0) * 1000),(gripPosition(2,0) * 1000), 0.0f, 0.0f, -180.0f };
	//	if ((gripDestination[0, 2] < 2)) gripDestination[0, 2] == 2;
	//	soc.moveRobotTo(gripDestination, ABS_LINEAR_SLOW);

	//	std::cout << "- GRIP" << std::endl;
	//	soc.moveRobotTo(gripDestination, COMPRESSED_AIR_9);

	//	std::cout << "- Move to drop position" << std::endl;
	//	float liftDestination[6] = { (gripPosition(0,0) * 1000), (gripPosition(1,0) * 1000), (150),0.0f, 0.0f, -180.0f };
	//	soc.moveRobotTo(liftDestination, ABS_LINEAR);
	//	//while (!soc.moveRobotTo(camera0Position, ABS_LINEAR));

	//	//std::vector<double> dropDestination = { -210, -600, 2.5, -90, 0, 180};
	//	std::vector<double> dropDestination = { -140.62, -466.02, 108.68, 1.82, 0.16, 176.84 };

	//	dropDestination[0] += 110 * std::floor(objectsPicked / 3);
	//	dropDestination[1] += 110 * (objectsPicked % 3);
	//	objectsPicked++;

	//	std::vector<double> prepDropDestination;
	//	for (int i=0; i<dropDestination.size(); i++) 
	//		prepDropDestination.push_back(dropDestination[i]); 
	//	prepDropDestination[2] = 50.0;
	//	soc.moveRobotTo(prepDropDestination, ABS_LINEAR_150);

	//	std::vector<double> prepprepDropDestination;
	//	for (int i=0; i<dropDestination.size(); i++) 
	//	prepprepDropDestination.push_back(dropDestination[i]); 
	//	prepprepDropDestination[2] = 15.0;
	//	soc.moveRobotTo(prepprepDropDestination, ABS_LINEAR);
	//	
	//	soc.moveRobotTo(dropDestination, ABS_LINEAR_SLOW);
	//	std::cout << "- DROP" << std::endl;
	//	soc.moveRobotTo(dropDestination, COMPRESSED_AIR_10);
	//	std::cout << "- Move to READY position" << std::endl;
	//	soc.moveRobotTo(prepDropDestination, ABS_LINEAR);
	//	soc.moveRobotTo(preCamera0Position, ABS_LINEAR_150);
	//	//soc.moveRobotTo(camera0Position, ABS_LINEAR);
	//	std::this_thread::sleep_for(std::chrono::microseconds(500));

	return true;
	else 
	return false;
	}

	//int count;
	//count = 0;
	//if (count == 0)
	//{	//Gap
	//	std::cout << "- Move to gripping position " << std::endl;
	//	soc.moveRobotTo(prePick1, ABS_JOINT);
	//	soc.moveRobotTo(Pick1, ABS_LINEAR_150);
	//	std::cout << "- Open Compressed Air " << std::endl;
	//	soc.moveRobotTo(Pick1, COMPRESSED_AIR_9);
	//	soc.moveRobotTo(aftPick1, ABS_JOINT);
	//	std::cout << "- Move to Placing position " << std::endl;
	//	soc.moveRobotTo(prePlace1, ABS_LINEAR_150);
	//	soc.moveRobotTo(Place1, ABS_LINEAR_150);
	//	soc.moveRobotTo(Place1, COMPRESSED_AIR_10);

	//	soc.moveRobotTo(preCamera0Position, ABS_LINEAR_150);
	//	//soc.moveRobotTo(camera0Position, ABS_LINEAR);
	//	soc.moveRobotTo(camera0Position, COMPRESSED_AIR_10);

	//	count++;
	//}
	//std::cout << "count = " << count << std::endl;
	//return false;
	//if (count == 1)
	//{
	////Gap
	//	//std::cout << "- Move to gripping position " << std::endl;
	//	//soc.moveRobotTo(prePick2, ABS_JOINT);
	//	//soc.moveRobotTo(Pick2, ABS_LINEAR_150);
	//	//std::cout << "- Open Compressed Air " << std::endl;
	//	//soc.moveRobotTo(Pick2, COMPRESSED_AIR_9);
	//	//soc.moveRobotTo(aftPick1, ABS_JOINT);
	//	//std::cout << "- Move to Placing position " << std::endl;
	//	//soc.moveRobotTo(prePlace2, ABS_LINEAR_150);
	//	//soc.moveRobotTo(Place2, ABS_LINEAR_150);
	//	//soc.moveRobotTo(Place2, COMPRESSED_AIR_10);
	//	//soc.moveRobotTo(preCamera0Position, ABS_LINEAR_150);
	//	////soc.moveRobotTo(camera0Position, ABS_LINEAR);
	//	//soc.moveRobotTo(camera0Position, COMPRESSED_AIR_10);
	//	count++;
	//}
	//std::cout << "count = " << count << std::endl;
	//return false;
//}

void RobotCommunication::startSequence()
{

}

