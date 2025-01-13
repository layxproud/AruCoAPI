#include "yamlhandler.h"
#include <QDebug>
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
    try {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened())
            return false;

        cv::FileNode configsNode = fs["Configurations"];
        for (const auto &configNode : configsNode) {
            Configuration config;
            configNode["ID"] >> config.id;
            configNode["Name"] >> config.name;
            configNode["Type"] >> config.type;
            configNode["Date"] >> config.date;
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
    } catch (const cv::Exception &e) {
        std::cerr << "OpenCV exception caught: " << e.what() << std::endl;
        return false;
    } catch (const std::exception &e) {
        std::cerr << "Standard exception caught: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
        return false;
    }
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
        fs << "ID" << config.second.id;
        fs << "Name" << config.second.name;
        fs << "Type" << config.second.type;
        fs << "Date" << config.second.date;
        fs << "MarkerIds"
           << "[";
        for (int id : config.second.markerIds) {
            fs << id;
        }
        fs << "]";
        fs << "RelativePoints"
           << "{";
        for (const auto &relativePoint : config.second.relativePoints) {
            fs << ("Marker_" + std::to_string(relativePoint.first)) << relativePoint.second;
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
            tr("Aborting operation, intersection of configurations is detected! Please make sure "
               "to save block with unique name and markers."));
        return false;
    }

    if (!saveConfigurations(filename, existingConfigurations)) {
        emit taskFinished(false, tr("Error occured while saving configuration file!"));
        return false;
    }

    emit taskFinished(true, tr("Configuration file updated successfully."));
    return true;
}

bool YamlHandler::removeConfiguration(
    const std::string &filename, const Configuration &configToRemove)
{
    std::map<std::string, Configuration> existingConfigurations;
    loadConfigurations(filename, existingConfigurations);

    auto it = existingConfigurations.find(configToRemove.name);
    if (it == existingConfigurations.end()) {
        emit taskFinished(false, tr("Could not find selected configuration!"));
        return false;
    }

    existingConfigurations.erase(it);

    if (!saveConfigurations(filename, existingConfigurations)) {
        emit taskFinished(false, tr("Error occured while saving configuration file!"));
        return false;
    }

    emit taskFinished(true, tr("Configuration deleted successfully."));
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
        // Searching matching names
        if (entry.second.name == currentConfiguration.name) {
            nameMatches = true;
            nameMatchConfig = entry.first;
        }

        // Converting to desired format
        std::vector<int> sortedExistingMarkerIds(entry.second.markerIds);
        std::vector<int> sortedCurrentMarkerIds(currentConfiguration.markerIds);

        std::sort(sortedExistingMarkerIds.begin(), sortedExistingMarkerIds.end());
        std::sort(sortedCurrentMarkerIds.begin(), sortedCurrentMarkerIds.end());

        // Searching for markers
        if (sortedExistingMarkerIds == sortedCurrentMarkerIds) {
            markerIdMatches = true;
            markerIdMatchConfig = entry.first;
        }

        // Searching for intersections
        std::vector<int> intersection;
        std::set_intersection(
            sortedExistingMarkerIds.begin(),
            sortedExistingMarkerIds.end(),
            sortedCurrentMarkerIds.begin(),
            sortedCurrentMarkerIds.end(),
            std::back_inserter(intersection));

        // Calculating result
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
