#include "arucoapi.h"
#include <QDebug>

AruCoAPI::AruCoAPI(QObject *parent)
    : QObject{parent}
    , yamlHandler(new YamlHandler(this))
    , captureThread(new CaptureThread(this))
{
    connect(captureThread, &CaptureThread::frameReady, this, &AruCoAPI::frameReady);
    connect(captureThread, &CaptureThread::distanceCalculated, this, &AruCoAPI::distanceCalculated);
    connect(captureThread, &CaptureThread::centerFound, this, &AruCoAPI::centerFound);
    connect(captureThread, &CaptureThread::newConfiguration, this, &AruCoAPI::newConfiguration);
}

AruCoAPI::~AruCoAPI()
{
    stopThread(captureThread);
}

void AruCoAPI::init()
{
    CalibrationParams calibrationParams;
    if (!yamlHandler->loadCalibrationParameters("calibration.yml", calibrationParams)) {
        emit taskFinished(false, tr("Не обнаружен файл калибровки. Сначала откалибруйте камеру!"));
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

void AruCoAPI::startMarkerDetectionTask()
{
    if (!captureThread->getMarkerDetectionStatus()) {
        captureThread->setDistanceCalculatingStatus(false);
        captureThread->setCenterFindingStatus(false);
        captureThread->setMarkerDetectingStatus(true);
        emit taskChanged(tr("Ведется поиск маркеров"));
    }
}

void AruCoAPI::startDistanceCalculationTask()
{
    if (!captureThread->getDistanceCalculatingStatus()) {
        captureThread->setCenterFindingStatus(false);
        captureThread->setMarkerDetectingStatus(false);
        captureThread->setDistanceCalculatingStatus(true);
        emit taskChanged(tr("Ведется измерение расстояний"));
    }
}

void AruCoAPI::startCenterFindingTask()
{
    if (!captureThread->getCenterFindingStatus()) {
        captureThread->setDistanceCalculatingStatus(false);
        captureThread->setMarkerDetectingStatus(false);
        captureThread->setCenterFindingStatus(true);
        emit taskChanged(tr("Ведется поиск центра"));
    }
}

void AruCoAPI::cancelOperations()
{
    captureThread->setDistanceCalculatingStatus(false);
    captureThread->setCenterFindingStatus(false);
    captureThread->setMarkerDetectingStatus(false);
    emit taskChanged(tr("Нет задачи"));
}
