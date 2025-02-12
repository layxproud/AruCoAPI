#ifndef TESTLIB_H
#define TESTLIB_H

#include "AruCoAPI_global.h"
#include "markerthread.h"
#include "yamlhandler.h"
#include <opencv2/opencv.hpp>
#include <QObject>

class ARUCOAPI_EXPORT AruCoAPI : public QObject
{
    Q_OBJECT

    void init();

public:
    explicit AruCoAPI(QObject *parent = nullptr);
    ~AruCoAPI();

    void startThread(QThread *thread);
    void stopThread(QThread *thread);

signals:
    void taskChanged(const QString &newTask); // Informs about changes to current task
    void taskFinished(bool success,
                      const QString &message);    // Informs about task completion status
    void frameReady(const QPixmap &pixmap);       // Frame captured by captureThread
    void blockDetected(const MarkerBlock &block); // Valid marker block detected

public slots:
    void detectMarkerBlocks(bool status); // Starts and ends block detection task

private:
    YamlHandler *yamlHandler;
    MarkerThread *markerThread;

    CalibrationParams calibrationParams;
    bool calibrationStatus;
};

#endif // TESTLIB_H
