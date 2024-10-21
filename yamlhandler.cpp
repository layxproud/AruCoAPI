#include "yamlhandler.h"
#include <QFile>

YamlHandler::YamlHandler(QObject *parent)
    : QObject(parent)
{}

bool YamlHandler::loadCalibrationParameters(const std::string &filename, CalibrationParams &params)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["CameraMatrix"] >> params.cameraMatrix;
    fs["DistCoeffs"] >> params.distCoeffs;
    fs.release();
    return true;
}

bool YamlHandler::saveCalibrationParameters(
    const std::string &filename, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
        return false;
    fs << "CameraMatrix" << cameraMatrix;
    fs << "DistCoeffs" << distCoeffs;
    fs.release();
    return true;
}

bool YamlHandler::loadConfigurations(
    const std::string &filename, std::map<std::string, Configuration> &configurations)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;

    cv::FileNode configsNode = fs["Configurations"];
    for (const auto &configNode : configsNode) {
        Configuration config;
        configNode["Name"] >> config.name;
        configNode["MarkerIds"] >> config.markerIds;
        cv::FileNode relativePointsNode = configNode["RelativePoints"];
        for (const auto &relativePointNode : relativePointsNode) {
            int markerId = std::stoi(relativePointNode.name().substr(7));
            relativePointNode >> config.relativePoints[markerId];
        }
        configurations.insert(std::make_pair(config.name, config));
    }
    fs.release();
    return true;
}

bool YamlHandler::saveConfigurations(
    const std::string &filename, const std::map<std::string, Configuration> &configurations)
{
    QFile::remove(QString::fromStdString(filename));

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
        return false;

    fs << "Configurations"
       << "[";
    for (const auto &config : configurations) {
        fs << "{";
        fs << "Name" << config.second.name;
        fs << "MarkerIds"
           << "[";
        for (int id : config.second.markerIds) {
            fs << id;
        }
        fs << "]";
        fs << "RelativePoints"
           << "{";
        for (const auto &relativePoint : config.second.relativePoints) {
            fs << "Marker_" + std::to_string(relativePoint.first) << relativePoint.second;
        }
        fs << "}";
        fs << "}";
    }
    fs << "]";
    fs.release();
    return true;
}

bool YamlHandler::updateConfigurations(
    const std::string &filename, const Configuration &currentConfiguration)
{
    std::map<std::string, Configuration> existingConfigurations;
    loadConfigurations(filename, existingConfigurations);

    std::string duplicateName;
    ConflictType conflict
        = findDuplicateConfiguration(existingConfigurations, currentConfiguration, duplicateName);

    switch (conflict) {
    case ConflictType::None:
        existingConfigurations.insert(
            std::make_pair(currentConfiguration.name, currentConfiguration));
        break;
    case ConflictType::ExactMatch:
        existingConfigurations[duplicateName] = currentConfiguration;
        break;
    case ConflictType::Intersection:
        emit taskFinished(
            false,
            tr("Обнаружено пересечение конфигураций! "
               "Пожалуйста, проверьте выбранное название и "
               "маркеры."));
        return false;
    }

    if (!saveConfigurations(filename, existingConfigurations)) {
        emit taskFinished(false, tr("Не удалось сохранить файл конфигураций!"));
        return false;
    }

    emit taskFinished(true, tr("Файл конфигураций успешно обновлен!"));
    return true;
}

ConflictType YamlHandler::findDuplicateConfiguration(
    const std::map<std::string, Configuration> &configurations,
    const Configuration &currentConfiguration,
    std::string &duplicateName)
{
    bool nameMatches = false;
    bool markerIdMatches = false;
    std::string nameMatchConfig;
    std::string markerIdMatchConfig;

    for (const auto &entry : configurations) {
        // Проверяем имена
        if (entry.second.name == currentConfiguration.name) {
            nameMatches = true;
            nameMatchConfig = entry.first;
        }

        // Конвертируем в адекватный формат
        std::vector<int> sortedExistingMarkerIds(entry.second.markerIds);
        std::vector<int> sortedCurrentMarkerIds(currentConfiguration.markerIds);

        std::sort(sortedExistingMarkerIds.begin(), sortedExistingMarkerIds.end());
        std::sort(sortedCurrentMarkerIds.begin(), sortedCurrentMarkerIds.end());

        // Проверяем маркеры
        if (sortedExistingMarkerIds == sortedCurrentMarkerIds) {
            markerIdMatches = true;
            markerIdMatchConfig = entry.first;
        }

        // Ищем пересечения
        std::vector<int> intersection;
        std::set_intersection(
            sortedExistingMarkerIds.begin(),
            sortedExistingMarkerIds.end(),
            sortedCurrentMarkerIds.begin(),
            sortedCurrentMarkerIds.end(),
            std::back_inserter(intersection));

        // Проверяем результат
        if (!intersection.empty() && intersection.size() != sortedExistingMarkerIds.size()) {
            duplicateName = "";
            return ConflictType::Intersection;
        }
    }

    if (nameMatches && !markerIdMatches) {
        duplicateName = nameMatchConfig;
        return ConflictType::ExactMatch;
    } else if (!nameMatches && markerIdMatches) {
        duplicateName = markerIdMatchConfig;
        return ConflictType::ExactMatch;
    } else if (nameMatches && markerIdMatches && nameMatchConfig == markerIdMatchConfig) {
        duplicateName = nameMatchConfig;
        return ConflictType::ExactMatch;
    } else if (nameMatches && markerIdMatches && nameMatchConfig != markerIdMatchConfig) {
        return ConflictType::Intersection;
    }

    return ConflictType::None;
}
