#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include "yamlhandler.h"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <QMutex>
#include <QThread>

class CaptureThread : public QThread
{
    Q_OBJECT
public:
    CaptureThread(QObject *parent = nullptr);

    void setYamlHandler(YamlHandler *handler) { yamlHandler = handler; }
    void setCalibrationParams(const CalibrationParams &params) { calibrationParams = params; }
    void setMarkerSize(float newSize) { markerSize = newSize; }
    void setMarkerDetectingStatus(bool status) { markerDetectionStatus = status; }
    void setDistanceCalculatingStatus(bool status) { distanceCalculatingStatus = status; }
    void setCenterFindingStatus(bool status) { centerFindingStatus = status; }

    bool getMarkerDetectionStatus() { return markerDetectionStatus; }
    bool getDistanceCalculatingStatus() { return distanceCalculatingStatus; }
    bool getCenterFindingStatus() { return centerFindingStatus; }

    void stop();

signals:
    void frameReady(const cv::Mat &frame);
    void distanceCalculated(const QVector<QPair<int, double>> &markers);
    void centerFound(double distance);

protected:
    void run() override;

private:
    bool running;
    cv::Mat currentFrame;
    cv::VideoCapture cap;
    QMutex mutex;
    bool markerDetectionStatus;
    bool distanceCalculatingStatus;
    bool centerFindingStatus;
    float markerSize;
    YamlHandler *yamlHandler;

    CalibrationParams calibrationParams;
    cv::aruco::Dictionary AruCoDict;
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector;

    // Обнаружение маркеров
    cv::Mat objPoints;
    std::vector<int> markerIds;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;

    // Поиск центра
    Configuration currentConfiguration;
    std::map<std::string, Configuration> configurations;
    cv::Point3f centerPoint;

private:
    // Измерение расстояний
    void calculateDistance();

    // Поиск центра
    void findAndDrawCenter();
    void updateConfigurationsMap();
    void detectCurrentConfiguration();
    void updateCenterPointPosition();
    cv::Point3f calculateMedianPoint(const std::vector<cv::Point3f> &points);
};

#endif // CAPTURETHREAD_H
