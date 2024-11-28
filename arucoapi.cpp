#include "arucoapi.h"
#include <QDebug>

AruCoAPI::AruCoAPI(QObject *parent)
    : QObject{parent}
    , yamlHandler(new YamlHandler(this))
    , captureThread(new CaptureThread(this))
{
    connect(captureThread, &CaptureThread::frameCaptured, this, &AruCoAPI::frameCaptured);
    connect(captureThread, &CaptureThread::blockDetected, this, &AruCoAPI::blockDetected);
    init();
}

AruCoAPI::~AruCoAPI()
{
    stopThread(captureThread);
}

void AruCoAPI::init()
{
    CalibrationParams calibrationParams;
    if (!yamlHandler->loadCalibrationParameters("calibration.yml", calibrationParams)) {
        emit taskFinished(false, tr("No calibration file found. Calibrate your camera first!"));
        return;
    }
    captureThread->setCalibrationParams(calibrationParams);
    captureThread->setYamlHandler(yamlHandler);
    startThread(captureThread);
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
        CaptureThread *capture = dynamic_cast<CaptureThread *>(thread);
        if (capture) {
            capture->stop();
            capture->wait();
            return;
        }

        thread->quit();
        thread->wait();
    }
}

void AruCoAPI::detectMarkerBlocks(bool status)
{
    if (status) {
        if (!captureThread->getBlockDetectionStatus()) {
            captureThread->setBlockDetectionStatus(true);
            emit taskChanged(tr("Block detection started"));
        }
    } else {
        if (captureThread->getBlockDetectionStatus()) {
            captureThread->setBlockDetectionStatus(false);
            emit taskChanged(tr("Block detection stopped"));
        }
    }
}
