#include "markerthread.h"
#include <QDebug>
#include <QPointF>

MarkerThread::MarkerThread(QObject *parent)
    : QThread{parent}
    , running(false)
    , markerSize(55.0f)
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

void MarkerThread::stop()
{
    QMutexLocker locker(&mutex);
    running = false;
}

void MarkerThread::run()
{
    cv::Mat resizedImage;
    cv::Size newSize(640, 480);

    cap.open(0);
    // cap.open(
    //     "udpsrc port=5000 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, "
    //     "encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! "
    //     "appsink",
    //     cv::CAP_GSTREAMER);

    if (!cap.isOpened())
        return;

    running = true;

    while (running) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
            continue;

        {
            QMutexLocker locker(&mutex);
            currentFrame = frame.clone();
            cv::resize(currentFrame, resizedImage, newSize);

            markerIds.clear();
            markerPoints.clear();
            rvecs.clear();
            tvecs.clear();

            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;
            detector.detectMarkers(resizedImage, markerCorners, markerIds, rejectedCorners);

            if (blockDetectionStatus && markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(resizedImage, markerCorners, markerIds);

                int nMarkers = markerCorners.size();
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

                    markerPoints.push_back(
                        std::make_pair(markerCorners[i][0], cv::Point3f(tvecs[i])));
                }

                updateSelectedPointPosition();

                // 3D point to 2D
                if (selectedPoint != cv::Point3f(0.0, 0.0, 0.0)
                    && !currentConfiguration.name.empty()) {
                    qDebug() << "X: " << selectedPoint.x;
                    qDebug() << "Y: " << selectedPoint.y;
                    qDebug() << "Distance: " << selectedPoint.z;
                    std::vector<cv::Point3f> points3D = {selectedPoint};
                    std::vector<cv::Point2f> points2D;
                    cv::projectPoints(
                        points3D,
                        cv::Vec3d::zeros(),
                        cv::Vec3d::zeros(),
                        calibrationParams.cameraMatrix,
                        calibrationParams.distCoeffs,
                        points2D);

                    cv::circle(resizedImage, points2D[0], 5, cv::Scalar(0, 0, 255), -1);

                    std::stringstream ss;
                    double distance = std::sqrt(
                        selectedPoint.x * selectedPoint.x + selectedPoint.y * selectedPoint.y
                        + selectedPoint.z * selectedPoint.z);
                    ss << "DISTANCE: " << distance << " mm";
                    cv::putText(
                        resizedImage,
                        ss.str(),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1,
                        cv::Scalar(0, 255, 0),
                        2);
                }
            } else {
                detectCurrentConfiguration();
            }
            emit frameReady(resizedImage);
        }
    }

    cap.release();
}

void MarkerThread::onPointSelected(const QPointF &point)
{
    QMutexLocker locker(&mutex);

    if (markerIds.size() != 4) {
        emit taskFinished(false, tr("You need exactly 4 markers to create a configuration"));
        return;
    }

    cv::Point2f clickedPoint2D(point.x(), point.y());

    float depth = getDepthAtPoint(clickedPoint2D);
    cv::Point3f clickedPoint3D = projectPointTo3D(clickedPoint2D, depth);

    Configuration newConfig = Configuration{};
    newConfig.markerIds = markerIds;
    newConfig.name = currentConfiguration.name.empty() ? "New configuration"
                                                       : currentConfiguration.name;

    for (int markerId : newConfig.markerIds) {
        auto it = std::find(markerIds.begin(), markerIds.end(), markerId);
        if (it != markerIds.end()) {
            int index = std::distance(markerIds.begin(), it);
            cv::Point3f relativePoint
                = calculateRelativePosition(clickedPoint3D, rvecs.at(index), tvecs.at(index));
            newConfig.relativePoints[markerId] = relativePoint;
        }
    }

    configurations[newConfig.name] = newConfig;
    // This code was created to help GUI correctly update
    Configuration tempConfig = newConfig;
    detectCurrentConfiguration();
    if (tempConfig.name == currentConfiguration.name) {
        currentConfiguration = tempConfig;
    }
}

void MarkerThread::updateConfigurationsMap()
{
    configurations.clear();
    currentConfiguration.clear();
    yamlHandler->loadConfigurations("configurations.yml", configurations);
}

void MarkerThread::detectCurrentConfiguration()
{
    Configuration new_Configuration = Configuration{};
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
        emit newConfiguration(new_Configuration);
        currentConfiguration = new_Configuration;
    }
}

cv::Vec4f MarkerThread::calculateMarkersPlane(const std::vector<cv::Point3f> &marker3DPoints)
{
    cv::Point3f v1 = marker3DPoints[1] - marker3DPoints[0];
    cv::Point3f v2 = marker3DPoints[2] - marker3DPoints[0];

    cv::Point3f normal = v1.cross(v2);
    float D = -(normal.dot(marker3DPoints[0]));

    return cv::Vec4f(normal.x, normal.y, normal.z, D);
}

float MarkerThread::getDepthAtPoint(const cv::Point2f &point)
{
    cv::Mat K = calibrationParams.cameraMatrix;
    float x = (point.x - K.at<double>(0, 2)) / K.at<double>(0, 0);
    float y = (point.y - K.at<double>(1, 2)) / K.at<double>(1, 1);
    cv::Point3f rayDir(x, y, 1.0f);

    std::vector<cv::Point3f> marker3DPoints;
    for (const auto &marker : markerPoints) {
        marker3DPoints.push_back(marker.second);
    }

    cv::Vec4f plane = calculateMarkersPlane(marker3DPoints);

    float numerator = -(plane[0] * 0 + plane[1] * 0 + plane[2] * 0 + plane[3]);
    float denominator = plane[0] * rayDir.x + plane[1] * rayDir.y + plane[2] * rayDir.z;

    float t = numerator / denominator;

    return t;
}

cv::Point3f MarkerThread::projectPointTo3D(const cv::Point2f &point2D, float depth)
{
    float x = (point2D.x - calibrationParams.cameraMatrix.at<double>(0, 2))
              / calibrationParams.cameraMatrix.at<double>(0, 0);
    float y = (point2D.y - calibrationParams.cameraMatrix.at<double>(1, 2))
              / calibrationParams.cameraMatrix.at<double>(1, 1);

    return cv::Point3f(x * depth, y * depth, depth);
}

cv::Point3f MarkerThread::calculateRelativePosition(
    const cv::Point3f &point3D, const cv::Vec3d &rvec, const cv::Vec3d &tvec)
{
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);

    cv::Mat rotationMatrixInv = rotationMatrix.inv();
    cv::Mat tvecInv = -rotationMatrixInv * cv::Mat(tvec);

    cv::Mat pointMat = (cv::Mat_<double>(3, 1) << point3D.x, point3D.y, point3D.z);
    cv::Mat relativePointMat = rotationMatrixInv * pointMat + tvecInv;

    return cv::Point3f(
        relativePointMat.at<double>(0),
        relativePointMat.at<double>(1),
        relativePointMat.at<double>(2));
}

void MarkerThread::updateSelectedPointPosition()
{
    detectCurrentConfiguration();
    if (currentConfiguration.name.empty()) {
        return;
    }
    const auto &config = currentConfiguration;
    std::vector<cv::Point3f> allPoints;
    std::vector<float> reprojectionErrors;

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

            float error = cv::norm(relativePointMat - newPointMat);

            allPoints.push_back(cv::Point3f(
                newPointMat.at<double>(0), newPointMat.at<double>(1), newPointMat.at<double>(2)));
            reprojectionErrors.push_back(error);
        }
    }

    if (!allPoints.empty()) {
        selectedPoint = calculateWeightedAveragePoint(allPoints, reprojectionErrors);
    }
}

cv::Point3f MarkerThread::calculateWeightedAveragePoint(
    const std::vector<cv::Point3f> &points, const std::vector<float> &errors)
{
    cv::Point3f weightedSum(0, 0, 0);
    float totalWeight = 0.0f;

    for (size_t i = 0; i < points.size(); i++) {
        float weight = 1.0f / (errors[i] + 1e-5);
        weightedSum += points[i] * weight;
        totalWeight += weight;
    }

    return weightedSum / totalWeight;
}
