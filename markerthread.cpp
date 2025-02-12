#include "markerthread.h"
#include <QDebug>
#include <QPointF>

MarkerThread::MarkerThread(QObject *parent)
    : QThread{parent}
    , running(false)
    , blockDetectionStatus(false)
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
    cv::Mat resizedFrame;
    cv::Size newSize(640, 480);

    cap.open(0);

    if (!cap.isOpened()) {
        emit taskFinished(false, tr("Failed to open camera"));
        cap.release();
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
            cv::resize(currentFrame, resizedFrame, newSize);

            markerIds.clear();
            markerPoints.clear();
            rvecs.clear();
            tvecs.clear();

            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;
            detector.detectMarkers(resizedFrame, markerCorners, markerIds, rejectedCorners);

            if (blockDetectionStatus && markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(resizedFrame, markerCorners, markerIds);

                int nMarkers = markerCorners.size();
                rvecs.resize(nMarkers);
                tvecs.resize(nMarkers);

                std::vector<float> yaws{}; // Stores yaw angles of markers

                for (size_t i = 0; i < nMarkers; i++) {
                    solvePnP(
                        objPoints,
                        markerCorners.at(i),
                        calibrationParams.cameraMatrix,
                        calibrationParams.distCoeffs,
                        rvecs.at(i),
                        tvecs.at(i));

                    cv::Mat rotationMatrix;
                    cv::Rodrigues(rvecs.at(i), rotationMatrix);

                    // Calculate yaw angle and normalize to [0, 360)
                    float yaw
                        = atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0))
                          * (180.0 / CV_PI);
                    if (yaw < 0) {
                        yaw += 360.0f;
                    }

                    yaws.push_back(yaw);

                    markerPoints.push_back(
                        std::make_pair(markerCorners[i][0], cv::Point3f(tvecs[i])));
                }

                updateCenterPointPosition();

                MarkerBlock block{};

                // 3D point to 2D
                if (centerPoint != cv::Point3f(0.0, 0.0, 0.0)
                    && !currentConfiguration.name.empty()) {
                    std::vector<cv::Point3f> points3D = {centerPoint};
                    std::vector<cv::Point2f> points2D;
                    cv::projectPoints(
                        points3D,
                        cv::Vec3d::zeros(),
                        cv::Vec3d::zeros(),
                        calibrationParams.cameraMatrix,
                        calibrationParams.distCoeffs,
                        points2D);

                    cv::circle(resizedFrame, points2D[0], 5, cv::Scalar(0, 0, 255), -1);

                    float distanceToCenter = std::sqrt(
                        centerPoint.x * centerPoint.x + centerPoint.y * centerPoint.y
                        + centerPoint.z * centerPoint.z);
                    block.blockCenter = QPointF(centerPoint.x, centerPoint.y);
                    block.distanceToCenter = distanceToCenter;
                    block.config = currentConfiguration;

                    if (!yaws.empty()) {
                        std::sort(yaws.begin(), yaws.end());
                        size_t mid = yaws.size() / 2;
                        block.blockAngle = (yaws.size() % 2 == 0) ? (yaws[mid - 1] + yaws[mid]) / 2.0f
                                                                  : yaws[mid];
                    }
                    emit blockDetected(block);
                } 
            } else {
                detectCurrentConfiguration();
            }
            QImage
                img(resizedFrame.data,
                    resizedFrame.cols,
                    resizedFrame.rows,
                    resizedFrame.step,
                    QImage::Format_RGB888);
            QPixmap pixmap = QPixmap::fromImage(img.rgbSwapped());
            emit frameReady(pixmap);
        }
    }

    cap.release();
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

void MarkerThread::updateCenterPointPosition()
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
        centerPoint = calculateWeightedAveragePoint(allPoints, reprojectionErrors);
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
