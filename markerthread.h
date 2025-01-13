#ifndef MARKERTHREAD_H
#define MARKERTHREAD_H

#include "yamlhandler.h"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <QMutex>
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
    void frameReady(const cv::Mat &frame);
    void blockDetected(const MarkerBlock &block);
    void newConfiguration(const Configuration &config);
    void taskFinished(bool success, const QString &message);

protected:
    void run() override;

public slots:
    void onPointSelected(const QPointF &point);
    void setMarkerSize(int size) { markerSize = (float) size; }
    void updateConfigurationsMap();

private:
    YamlHandler *yamlHandler;
    bool running;
    cv::Mat currentFrame;
    cv::VideoCapture cap;
    QMutex mutex;
    bool blockDetectionStatus;
    float markerSize;

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

    cv::Point3f selectedPoint;

    void detectCurrentConfiguration();

    cv::Vec4f calculateMarkersPlane(const std::vector<cv::Point3f> &marker3DPoints);
    float getDepthAtPoint(const cv::Point2f &point);
    cv::Point3f projectPointTo3D(const cv::Point2f &point2D, float depth);
    cv::Point3f calculateRelativePosition(
        const cv::Point3f &point3D, const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    void updateSelectedPointPosition();
    cv::Point3f calculateWeightedAveragePoint(
        const std::vector<cv::Point3f> &points, const std::vector<float> &errors);
};

#endif // MARKERTHREAD_H
