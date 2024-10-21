#ifndef TESTLIB_H
#define TESTLIB_H

#include "AruCoAPI_global.h"
#include "capturethread.h"
#include "yamlhandler.h"
#include <opencv2/opencv.hpp>
#include <QObject>

class ARUCOAPI_EXPORT AruCoAPI : public QObject
{
    Q_OBJECT
public:
    explicit AruCoAPI(QObject *parent = nullptr);
    ~AruCoAPI();

    void init();
    void startThread(QThread *thread);
    void stopThread(QThread *thread);

signals:
    void frameReady(const cv::Mat &frame);
    void taskChanged(const QString &newTask);
    void taskFinished(bool success, const QString &message);
    void distanceCalculated(const QVector<QPair<int, double>> &markers);
    void centerFound(double distance);

public slots:
    void startMarkerDetectionTask();
    void startDistanceCalculationTask();
    void startCenterFindingTask();
    void cancelOperations();

private:
    YamlHandler *yamlHandler;
    CaptureThread *captureThread;
};

#endif // TESTLIB_H
