#include "Reader.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

Reader::Reader(const std::string& folder_path){
    folder_path_ = folder_path;
    // Check if the folder path exists
    if (isFolderNotEmpty(folder_path)) {
        std::cout << "The path " << folder_path << " exists and is not empty." << std::endl;
    }

}

std::string Reader::getFolderPath() const{
    return folder_path_;
}


// Function to read calibration pattern information from a YAML file
bool Reader::readCalibrationInfo(CalibrationInfo& calib_info) {
    try {
        YAML::Node config = YAML::LoadFile(this->getFolderPath() + "/CalibrationInfo.yaml");

        calib_info.setNumberOfCams(config["number_of_cameras"].as<int>());
        calib_info.setCamFolderPref(config["camera_folder_prefix"].as<std::string>());
        calib_info.setCalibSetup(config["calibration_setup"].as<int>());

        calib_info.printCalibInfo();

        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}


int Reader::countImagesInFolder(const std::string& folder_path) {
    int image_count = 0;
    const std::vector<std::string> image_extensions = { ".jpg", ".jpeg", ".png", ".gif", ".tiff" };

    try{
        fs::path folder(folder_path);

        if (fs::exists(folder) && fs::is_directory(folder)) {
            for (const auto& entry : fs::directory_iterator(folder)) {
                if (fs::is_regular_file(entry)) {
                    for (const std::string& extension : image_extensions) {
                        if (entry.path().extension() == extension) {
                            image_count++;
                            break;
                        }
                    }
                }
            }
        }
    }
    catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "An error occurred while accessing the folder: " << ex.what() << std::endl;
    }

    return image_count;
}


void Reader::readPosesFromJSON(const std::string& filename, std::vector<cv::Mat>& robot_poses, std::vector<cv::Mat>& camera_poses) {
    // Open the JSON file
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    // Parse the JSON
    json j;
    file >> j;

    // Loop through each entry in the JSON array
    for (const auto& entry : j) {
        // Read the robot pose
        cv::Mat robot_pose(4, 4, CV_64F);
        auto robot_matrix = entry["robot"];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                robot_pose.at<double>(i, j) = robot_matrix[i][j];
            }
        }
        robot_poses.push_back(robot_pose);

        // Read the camera pose
        cv::Mat camera_pose(4, 4, CV_64F);
        auto camera_matrix = entry["camera"];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                camera_pose.at<double>(i, j) = camera_matrix[i][j];
            }
        }
        camera_poses.push_back(camera_pose);
    }
}




// Function to read JSON file and extract data
void Reader::readPointsFromJSON(const std::string& file_path,
                        std::vector<cv::Point3f>& poses3d_world,
                        std::vector<std::vector<cv::Point2f>>& poses2d,
                        std::vector<CameraParams>& camera_params) {
    // Open and parse the JSON file
    std::ifstream file(this->getFolderPath() + file_path);
    json j;
    file >> j;

    // Extract poses3d_world
    for (const auto& point : j["poses3d_world"]) {
        poses3d_world.emplace_back(point[0], point[1], point[2]);
    }

    // Extract poses2d for each camera
    for (const auto& view : j["poses2d"]) {
        std::vector<cv::Point2f> camera_points;
        for (const auto& point : view) {
            camera_points.emplace_back(point[0], point[1]);
        }
        poses2d.push_back(camera_points);
    }

    // Extract camera parameters
    for (const auto& camera : j["camera_params"]) {
        CameraParams params;

        // Read intrinsic parameters
        params.intrinsic = cv::Mat::zeros(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                params.intrinsic.at<double>(i, j) = camera[0][i][j];
            }
        }

        // Read extrinsic parameters (4x4 transformation matrix)
        params.extrinsic = cv::Mat::zeros(4, 4, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                params.extrinsic.at<double>(i, j) = camera[1][i][j];
            }
        }
        params.extrinsic.at<double>(3, 3) = 1.0; // Set the bottom-right element to 1 for a valid transformation matrix

        // Read extrinsic parameters (4x4 transformation matrix)
        params.dist = cv::Mat::zeros(5, 1, CV_64F);
        for (int i = 0; i < 5; ++i) {
            params.dist.at<double>(i) = camera[2][i];
        }

        camera_params.push_back(params);
    }
}

// Function to create a YAML file for camera parameters
void Reader::writeCameraParamsToYAML(const std::string& file_path, const CameraParams& params) {
    std::ofstream file(file_path);

    file << "fx: " << params.intrinsic.at<double>(0, 0) << "\n";
    file << "fy: " << params.intrinsic.at<double>(1, 1) << "\n";
    file << "cx: " << params.intrinsic.at<double>(0, 2) << "\n";
    file << "cy: " << params.intrinsic.at<double>(1, 2) << "\n";
    file << "has_dist_coeff: 1\n";
    file << "dist_k0: " << params.dist.at<double>(0, 0) << "\n";
    file << "dist_k1: " << params.dist.at<double>(0, 1) << "\n";
    file << "dist_px: " << params.dist.at<double>(0, 2) << "\n";
    file << "dist_py: " << params.dist.at<double>(0, 3) << "\n";
    file << "dist_k2: " << params.dist.at<double>(0, 4) << "\n";
    if (params.dist.cols > 5) {
        file << "dist_k3: " << params.dist.at<double>(0, 5) << "\n";
        file << "dist_k4: " << params.dist.at<double>(0, 6) << "\n";
        file << "dist_k5: " << params.dist.at<double>(0, 7) << "\n";
    } else {
        file << "dist_k3: 0\n";
        file << "dist_k4: 0\n";
        file << "dist_k5: 0\n";
    }
    file << "img_width: 1000\n";  // Example image width
    file << "img_height: 1002\n"; // Example image height

    file.close();
}

void Reader::writePointsToFile(const std::string &filename, const std::vector<cv::Point3f> &optimal_3d_points) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    for (const auto &pt : optimal_3d_points) {
        file << pt.x << " " << pt.y << " " << pt.z << "\n";
    }

    file.close();
}