#include "Optimizer.h"
#include "CameraOptimizer.h"
#include <filesystem>

namespace fs = std::filesystem;

void Optimizer::optimization() {

    std::cout << "##################################################" << std::endl;
    std::cout << "START THE OPTIMIZATION PROCESS!" << std::endl;

    //------------------------------ DATA READING ------------------------------
    // Create the Calibration Info structure
    std::string dataset = getStringAfterLastSlash(data_);
    std::cout << "Dataset: " << dataset << std::endl;

    // Read the provided data
    Reader reader(data_);

    //###########################################################
    // HUMAN 3D POSE OPTIMIZATION
    std::vector<cv::Point3f> poses3d_world;
    std::vector<std::vector<cv::Point2f>> poses2d;
    std::vector<CameraParams> camera_params;

    reader.readPointsFromJSON(file_json_, poses3d_world, poses2d, camera_params);

    // Print the extracted data for verification
    /*std::cout << "Poses 3D World:" << std::endl;
    for (const auto& point : poses3d_world) {
        std::cout << point << std::endl;
    }


    for (size_t cam_idx = 0; cam_idx < poses2d.size(); ++cam_idx) {
        std::cout << "Poses 2D Camera " << cam_idx << ":" << std::endl;
        for (const auto& point : poses2d[cam_idx]) {
            std::cout << point << std::endl;
        }
    }


    std::cout << "Camera Parameters:" << std::endl;
    for (const auto& params : camera_params) {
        std::cout << "Intrinsic:" << std::endl;
        std::cout << params.intrinsic << std::endl;
        std::cout << "Extrinsic:" << std::endl;
        std::cout << params.extrinsic << std::endl;
        std::cout << "Dist:" << std::endl;
        std::cout << params.dist << std::endl;
    }*/


    std::vector<cv::Mat> world2cam_vec{camera_params.size()};
    std::vector<CameraInfo> camera_info{camera_params.size()};
    // Write camera parameters to YAML files
    for (size_t i = 0; i < camera_params.size(); ++i) {
        std::string intrinsic_path = "intrinsic_pars_file_" + std::to_string(i) + ".yaml";
        std::string yaml_file_path = data_ + intrinsic_path;
        reader.writeCameraParamsToYAML(yaml_file_path, camera_params[i]);
        std::cout << "Written camera parameters to " << yaml_file_path << std::endl;
        camera_info[i].setParameters(data_ + intrinsic_path);
        world2cam_vec[i] = camera_params[i].extrinsic;
    }
     
    std::vector<cv::Point3f> optimal_3d_points;
    CameraOptimizer camera_optimizer(poses3d_world, world2cam_vec);
    camera_optimizer.optimizer(camera_info, poses2d, optimal_3d_points);
    reader.writePointsToFile(data_ + "optimized_pattern_pts.txt", optimal_3d_points);
    
    //###########################################################
    
}
