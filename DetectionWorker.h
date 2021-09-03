#pragma once

#include <QObject>
#include <QThread>
#include "include/dataType/dataType.h"
#include "include/PPF/PPF.h"
#include "include/binSceneMaker/binSceneMaker.h"
#include "include/robotCommunication/robotCommunication.h"

class DetectionController : public QObject
{
    Q_OBJECT
    
public:
	QThread PPFWorkerThread;
	QThread simulationWorkerThread;
	QThread robotWorkerThread;

	DescriptorPPF *ppfWorker;
	BinSceneMaker *simulationWorker;
	RobotCommunication *robotWorker;
	//SimulationWorker *simuworker;

	DetectionController();
	~DetectionController();

public Q_SLOTS:
    void handleResults(const QString & result);
	void handleUiRequest(const QString & request);

Q_SIGNALS:
    void operate(const QString &);
	void updateUiFromController(const QString &);
	void operateDetection(const MyArray& pos);
};
