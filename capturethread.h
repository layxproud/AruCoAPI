#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include "yamlhandler.h"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <QMutex>
#include <QPixmap>
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

class CaptureThread : public QThread
{
    Q_OBJECT
public:
    CaptureThread(QObject *parent = nullptr);

    // Setters
    void setYamlHandler(YamlHandler *handler) { yamlHandler = handler; }
    void setCalibrationParams(const CalibrationParams &params) { calibrationParams = params; }
    void setMarkerSize(float newSize) { markerSize = newSize; }
    void setBlockDetectionStatus(bool status) { blockDetectionStatus = status; }

    // Getters
    bool getBlockDetectionStatus() { return blockDetectionStatus; }

    void stop(); // Stops the thread

signals:
    void frameCaptured(const QPixmap &frame);     // frame captured by VideoCapture
    void blockDetected(const MarkerBlock &block); // valid marker block detected
    void taskFinished(bool success,
                      const QString &message); // Informs about task completion status

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
    cv::Point3f calculateWeightedAveragePoint(
        const std::vector<cv::Point3f> &points, const std::vector<float> &errors);
};

#endif // CAPTURETHREAD_H
