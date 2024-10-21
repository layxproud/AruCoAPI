#include "capturethread.h"
#include <QDebug>

CaptureThread::CaptureThread(QObject *parent)
    : QThread(parent)
    , running(false)
    , markerDetectionStatus(false)
    , distanceCalculatingStatus(false)
    , centerFindingStatus(false)
    , markerSize(31.0f)
{
    updateConfigurationsMap();

    AruCoDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    detectorParams = cv::aruco::DetectorParameters();
    detector = cv::aruco::ArucoDetector(AruCoDict, detectorParams);

    objPoints = cv::Mat(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerSize / 2.f, markerSize / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerSize / 2.f, markerSize / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerSize / 2.f, -markerSize / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerSize / 2.f, -markerSize / 2.f, 0);
}

void CaptureThread::stop()
{
    QMutexLocker locker(&mutex);
    running = false;
}

void CaptureThread::run()
{
    cap.open(0);

    if (!cap.isOpened()) {
        qCritical() << "Не удалось открыть камеру";
        return;
    }

    running = true;

    while (running) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
            continue;

        {
            QMutexLocker locker(&mutex);
            currentFrame = frame.clone();

            markerCorners.clear();
            rejectedCorners.clear();
            markerIds.clear();
            rvecs.clear();
            tvecs.clear();

            detector.detectMarkers(frame, markerCorners, markerIds, rejectedCorners);

            if (markerIds.size() > 0) {
                if (markerDetectionStatus) {
                    cv::aruco::drawDetectedMarkers(currentFrame, markerCorners, markerIds);
                } else if (distanceCalculatingStatus) {
                    calculateDistance();
                } else if (centerFindingStatus) {
                    findAndDrawCenter();
                }
            } else {
                detectCurrentConfiguration();
            }
            emit frameReady(currentFrame);
        }
    }
    cap.release();
}

void CaptureThread::calculateDistance()
{
    if (markerIds.empty())
        return;

    int nMarkers = markerCorners.size();
    QVector<QPair<int, double>> markers;

    rvecs.clear();
    tvecs.clear();
    rvecs.resize(nMarkers);
    tvecs.resize(nMarkers);

    cv::aruco::estimatePoseSingleMarkers(
        markerCorners,
        markerSize,
        calibrationParams.cameraMatrix,
        calibrationParams.distCoeffs,
        rvecs,
        tvecs);

    for (size_t i = 0; i < markerIds.size(); ++i) {
        double distance = cv::norm(tvecs[i]);
        markers.append(qMakePair(markerIds.at(i), distance));
    }

    std::sort(
        markers.begin(),
        markers.end(),
        [](const QPair<int, double> &a, const QPair<int, double> &b) { return a.first < b.first; });

    emit distanceCalculated(markers);
}

void CaptureThread::findAndDrawCenter()
{
    int nMarkers = markerCorners.size();
    rvecs.clear();
    tvecs.clear();
    rvecs.resize(nMarkers);
    tvecs.resize(nMarkers);

    for (size_t i = 0; i < nMarkers; i++) {
        solvePnP(
            objPoints,
            markerCorners.at(i),
            calibrationParams.cameraMatrix,
            calibrationParams.distCoeffs,
            rvecs.at(i),
            tvecs.at(i));
    }

    updateCenterPointPosition();

    if (centerPoint != cv::Point3f(0.0, 0.0, 0.0) && !currentConfiguration.name.empty()) {
        std::vector<cv::Point3f> points3D = {centerPoint};
        std::vector<cv::Point2f> points2D;
        cv::projectPoints(
            points3D,
            cv::Vec3d::zeros(),
            cv::Vec3d::zeros(),
            calibrationParams.cameraMatrix,
            calibrationParams.distCoeffs,
            points2D);

        cv::circle(currentFrame, points2D[0], 5, cv::Scalar(0, 0, 255), -1);
        double distanceToCenter = std::sqrt(
            centerPoint.x * centerPoint.x + centerPoint.y * centerPoint.y
            + centerPoint.z * centerPoint.z);

        emit centerFound(distanceToCenter);
    }
}

void CaptureThread::updateConfigurationsMap()
{
    configurations.clear();
    currentConfiguration.name = "...---...";
    yamlHandler->loadConfigurations("configurations.yml", configurations);
}

void CaptureThread::detectCurrentConfiguration()
{
    Configuration new_Configuration;
    new_Configuration.name = "";
    for (const auto &config : configurations) {
        for (int id : config.second.markerIds) {
            if (std::find(markerIds.begin(), markerIds.end(), id) != markerIds.end()) {
                new_Configuration = config.second;
                break;
            }
        }
        if (!new_Configuration.name.empty()) {
            break;
        }
    }

    if (new_Configuration.name != currentConfiguration.name) {
        currentConfiguration = new_Configuration;
    }
}

void CaptureThread::updateCenterPointPosition()
{
    detectCurrentConfiguration();
    if (currentConfiguration.name.empty()) {
        return;
    }
    const auto &config = currentConfiguration;
    std::vector<cv::Point3f> allPoints;

    for (int id : config.markerIds) {
        auto it = std::find(markerIds.begin(), markerIds.end(), id);
        if (it != markerIds.end()) {
            int index = std::distance(markerIds.begin(), it);

            cv::Mat rotationMatrix;
            cv::Rodrigues(rvecs.at(index), rotationMatrix);

            cv::Mat relativePointMat
                = (cv::Mat_<double>(3, 1) << config.relativePoints.at(id).x,
                   config.relativePoints.at(id).y,
                   config.relativePoints.at(id).z);
            cv::Mat newPointMat = rotationMatrix * relativePointMat + cv::Mat(tvecs.at(index));

            allPoints.push_back(cv::Point3f(
                newPointMat.at<double>(0), newPointMat.at<double>(1), newPointMat.at<double>(2)));
        }
    }

    if (!allPoints.empty()) {
        cv::Point3f medianPoint = calculateMedianPoint(allPoints);
        centerPoint = medianPoint;
    }
}

cv::Point3f CaptureThread::calculateMedianPoint(const std::vector<cv::Point3f> &points)
{
    std::vector<cv::Point3f> sortedPoints = points;
    std::sort(sortedPoints.begin(), sortedPoints.end(), [](const cv::Point3f &a, const cv::Point3f &b) {
        return a.x < b.x;
    });

    size_t medianIndex = points.size() / 2;
    return sortedPoints[medianIndex];
}
