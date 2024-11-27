#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include "yamlhandler.h"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <QMutex>
#include <QThread>

class MarkerBlock
{
public:
    float x;
    float y;
    float z;
    Configuration config;
};

class CaptureThread : public QThread
{
    Q_OBJECT
public:
    CaptureThread(QObject *parent = nullptr);

    void setYamlHandler(YamlHandler *handler) { yamlHandler = handler; }
    void setCalibrationParams(const CalibrationParams &params) { calibrationParams = params; }
    void setMarkerSize(float newSize) { markerSize = newSize; }
    void setBlockDetectionStatus(bool status) { blockDetectionStatus = status; }

    bool getBlockDetectionStatus() { return blockDetectionStatus; }

    void stop();

signals:
    void blockDetected(const MarkerBlock &block, const QImage &frame);

protected:
    void run() override;

private:
    YamlHandler *yamlHandler;
    bool running;
    cv::Mat currentFrame;
    cv::VideoCapture cap;
    QMutex mutex;
    bool blockDetectionStatus;
    float markerSize;

    CalibrationParams calibrationParams;
    cv::aruco::Dictionary AruCoDict;
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector;

    // Marker Detection
    cv::Mat objPoints;
    std::vector<int> markerIds;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;

    // Block Detection
    Configuration currentConfiguration;
    std::map<std::string, Configuration> configurations;
    cv::Point3f centerPoint;

private:
    void detectBlock();
    void updateConfigurationsMap();
    void detectCurrentConfiguration();
    void updateCenterPointPosition();
    cv::Point3f calculateMedianPoint(const std::vector<cv::Point3f> &points);
};

#endif // CAPTURETHREAD_H
