#include "arucoapi.h"
#include <QDebug>

AruCoAPI::AruCoAPI(QObject *parent)
    : QObject{parent}
    , yamlHandler(new YamlHandler(this))
    , markerThread(new MarkerThread())
    , calibrationStatus(false)
{
    connect(markerThread, &MarkerThread::frameReady, this, &AruCoAPI::frameReady);
    connect(markerThread, &MarkerThread::blockDetected, this, &AruCoAPI::blockDetected);
    connect(markerThread, &MarkerThread::taskFinished, this, &AruCoAPI::taskFinished);
    connect(yamlHandler, &YamlHandler::taskFinished, this, &AruCoAPI::taskFinished);

    init();
}

AruCoAPI::~AruCoAPI()
{
    stopThread(markerThread);
}

void AruCoAPI::init()
{
    calibrationStatus = yamlHandler->loadCalibrationParameters("calibration.yml", calibrationParams);
    if (!calibrationStatus) {
        emit taskFinished(false, tr("No calibration file found. Calibrate your camera first!"));
        return;
    }
    markerThread->setCalibrationParams(calibrationParams);
    markerThread->setYamlHandler(yamlHandler);
    startThread(markerThread);
    qDebug() << "THREAD STARTED";
}

void AruCoAPI::startThread(QThread *thread)
{
    if (thread && !thread->isRunning()) {
        thread->start();
    }
}

void AruCoAPI::stopThread(QThread *thread)
{
    if (thread && thread->isRunning()) {
        MarkerThread *mthread = dynamic_cast<MarkerThread *>(thread);
        if (mthread) {
            mthread->stop();
            mthread->wait();
            return;
        }

        thread->quit();
        thread->wait();
    }
}

void AruCoAPI::detectMarkerBlocks(bool status)
{
    if (status) {
        if (!markerThread->getBlockDetectionStatus()) {
            markerThread->setBlockDetectionStatus(true);
            emit taskChanged(tr("Block detection started"));
        }
    } else {
        if (markerThread->getBlockDetectionStatus()) {
            markerThread->setBlockDetectionStatus(false);
            emit taskChanged(tr("Block detection stopped"));
        }
    }
}
