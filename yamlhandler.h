#ifndef YAMLHANDLER_H
#define YAMLHANDLER_H

#include <opencv2/opencv.hpp>
#include <QObject>

struct Configuration
{
    std::string id;
    std::string name;
    std::string type;
    std::string date;
    std::vector<int> markerIds;
    std::map<int, cv::Point3f> relativePoints;
};

struct CalibrationParams
{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
};

enum class ConflictType { None, ExactMatch, Intersection };

class YamlHandler : public QObject
{
    Q_OBJECT
public:
    YamlHandler(QObject *parent = nullptr);

    bool loadCalibrationParameters(const std::string &filename, CalibrationParams &params);
    bool saveCalibrationParameters(
        const std::string &filename, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs);
    bool loadConfigurations(
        const std::string &filename, std::map<std::string, Configuration> &configurations);
    bool saveConfigurations(
        const std::string &filename, const std::map<std::string, Configuration> &configurations);
    bool updateConfigurations(const std::string &filename, const Configuration &currentConfiguration);
    bool removeConfiguration(const std::string &filename, const Configuration &configToRemove);

signals:
    void taskFinished(bool success, const QString &message);

private:
    ConflictType findDuplicateConfiguration(
        const std::map<std::string, Configuration> &configurations,
        const Configuration &currentConfiguration,
        std::string &duplicateName);
};

#endif // YAMLHANDLER_H
