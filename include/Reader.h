#ifndef READER_H
#define READER_H

#include <string>
#include <filesystem>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>
#include <ctime>   // For time()

#include "utils.h"
#include "CalibrationInfo.h"
#include <json.hpp>

// Structure to hold camera parameters
struct CameraParams {
    cv::Mat intrinsic;
    cv::Mat extrinsic;
    cv::Mat dist;
};

class Reader{
private:

    std::string folder_path_;

public:
    Reader(const std::string& folder_path);
    std::string getFolderPath() const;
    bool readCalibrationInfo(CalibrationInfo& calib_info);
    void readPointsFromJSON(const std::string& file_path, std::vector<std::vector<cv::Point3f>>& poses3d_world, std::vector<std::vector<std::vector<cv::Point2f>>>& poses2d, std::vector<std::vector<CameraParams>>& camera_params);
    void writeCameraParamsToYAML(const std::string& file_path, const CameraParams& params);
    void writePointsToFile(const std::string &filename, const std::vector<cv::Point3f> &optimal_3d_points);
};

#endif //READER_H
