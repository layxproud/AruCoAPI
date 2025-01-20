#ifndef MARKERTHREAD_H
#define MARKERTHREAD_H

#include "yamlhandler.h"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <QMutex>
#include <QPixmap>
#include <QPointF>
#include <QThread>

// Class containing information about marker block
class MarkerBlock
{
public:
    QPointF blockCenter;
    float distanceToCenter;
    float blockAngle;
    Configuration config;
};

class MarkerThread : public QThread
{
    Q_OBJECT
public:
    explicit MarkerThread(QObject *parent = nullptr);

    void setYamlHandler(YamlHandler *handler) { yamlHandler = handler; }
    void setCalibrationParams(const CalibrationParams &params) { calibrationParams = params; }
    void setMarkerSize(float newSize) { markerSize = newSize; }
    void setBlockDetectionStatus(bool status) { blockDetectionStatus = status; }

    Configuration getCurrConfiguration() { return currentConfiguration; }
    bool getBlockDetectionStatus() { return blockDetectionStatus; }

    void stop();

signals:
    void frameReady(const QPixmap &pixmap);
    void blockDetected(const MarkerBlock &block);
    void newConfiguration(const Configuration &config);
    void taskFinished(bool success, const QString &message);

protected:
    void run() override;

public slots:
    void setMarkerSize(int size) { markerSize = (float) size; }
    void updateConfigurationsMap();

private:
    YamlHandler *yamlHandler;
    QMutex mutex;
    bool running;
    bool blockDetectionStatus;
    float markerSize;

    cv::Mat currentFrame;
    cv::VideoCapture cap;

    cv::aruco::Dictionary AruCoDict;
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector;
    cv::Mat objPoints;

    Configuration currentConfiguration;
    std::map<std::string, Configuration> configurations;

    CalibrationParams calibrationParams;
    std::vector<int> markerIds;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    std::vector<std::pair<cv::Point2f, cv::Point3f>> markerPoints;

    cv::Point3f centerPoint;

    void detectCurrentConfiguration();

    void updateCenterPointPosition();
    cv::Point3f calculateWeightedAveragePoint(
        const std::vector<cv::Point3f> &points, const std::vector<float> &errors);
};

#endif // MARKERTHREAD_H
