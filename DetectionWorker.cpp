#include "DetectionWorker.h"

//
//SimulationWorker::SimulationWorker()
//{
//	descr = new DescriptorPPF(true);
//	binSceneMaker = new BinSceneMaker();
//}
//
//void SimulationWorker::prepareModelDescriptor()
//{
//	descr->prepareModelDescriptor();
//}
//
//void SimulationWorker::_3D_Matching()
//{
//	bool MatchingResult = descr->_3D_Matching();
//	emit detectionResultReady(MatchingResult);
//}
//
//void SimulationWorker::makeBinScene()
//{
//	bool SimulationResult = binSceneMaker->makeBinScene();
//	emit simulationResultReady(SimulationResult);
//}

DetectionController::DetectionController()
{
	ppfWorker = new DescriptorPPF;
    ppfWorker->moveToThread(&PPFWorkerThread);
    connect(&PPFWorkerThread, SIGNAL(finished()), ppfWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(operate(QString)), ppfWorker, SLOT(doWork(QString)));
	//qRegisterMetaType<MyArray>("MyArray");
	connect(this, SIGNAL(operateDetection(MyArray)), ppfWorker, SLOT(detect(MyArray)));
    connect(ppfWorker, SIGNAL(resultReady(QString)), this, SLOT(handleResults(QString)));
	connect(ppfWorker, SIGNAL(updateUiFromWorker(QString)), this, SLOT(handleUiRequest(QString)));

	simulationWorker = new BinSceneMaker;
    simulationWorker->moveToThread(&simulationWorkerThread);
    connect(&simulationWorkerThread, SIGNAL(finished()), simulationWorker, SLOT(deleteLater()));
	connect(this, SIGNAL(operate(QString)), simulationWorker, SLOT(doWork(QString)));
    connect(simulationWorker, SIGNAL(resultReady(QString)), this, SLOT(handleResults(QString)));
	connect(simulationWorker, SIGNAL(updateUiFromWorker(QString)), this, SLOT(handleUiRequest(QString)));

	robotWorker = new RobotCommunication;
    robotWorker->moveToThread(&robotWorkerThread);
    connect(&robotWorkerThread, SIGNAL(finished()), robotWorker, SLOT(deleteLater()));
	connect(this, SIGNAL(operate(QString)), robotWorker, SLOT(doWork(QString)));
    connect(robotWorker, SIGNAL(resultReady(QString)), this, SLOT(handleResults(QString)));
}

DetectionController::~DetectionController() {
    PPFWorkerThread.quit();
	simulationWorkerThread.quit();
	robotWorkerThread.quit();

    PPFWorkerThread.wait();
    simulationWorkerThread.wait();
	robotWorkerThread.wait();
}

void DetectionController::handleResults(const QString & result)
{
	if (result == "prepareSuccess")
	{
		//std::cout << result.toStdString() << std::endl;
		return;
	}
	if (result == "loadMeshModelFileSuccess")
	{
		emit updateUiFromController("modelViewer");
		//std::cout << result.toStdString() << std::endl;
		return;
	}
	if (result == "detectObjectsSuccess")
	{
		//std::cout << result.toStdString() << std::endl;
		//robotWorker->loadObjectTransfromMatFromCam(ppfWorker->objectTransfromMatListFromCam);
		emit updateUiFromController("enableDetectionButton");
		return;
	}
	if (result == "SequenceDetectionSuccess")
	{
		//std::cout << result.toStdString() << std::endl;
		robotWorker->loadObjectTransfromMatFromCam(ppfWorker->objectTransfromMatListFromCam);
		emit robotWorker->doWork("pickObjectAndPlaceSequence");
		return;
	}
	if (result == "simulationDetectionSuccess")
	{
		//std::cout << result.toStdString() << std::endl;
		emit updateUiFromController("enableDetectionButton");
		return;
	}
	if (result == "makeBinSceneSuccess")
	{
		//std::cout << result.toStdString() << std::endl;
		emit updateUiFromController("enableDetectionButton");
		return;
	}
	if (result == "moveToCamera0Success")
	{
		emit updateUiFromController("moveToCamera0Done");
		//std::cout << result.toStdString() << std::endl;
		return;
	}
	if (result == "pickObjectAndPlaceSuccess")
	{
		emit updateUiFromController("robotCommunicationDone");
		//std::cout << result.toStdString() << std::endl;
		return;
	}
	if (result == "startSequenceSuccess")
	{
		emit updateUiFromController("robotCommunicationDone");
		//std::cout << result.toStdString() << std::endl;
		return;
	}
}

void DetectionController::handleUiRequest(const QString & request)
{
	emit updateUiFromController(request);
}


