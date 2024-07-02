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
    std::vector<std::vector<cv::Point3f>> poses3d_world;
    std::vector<std::vector<std::vector<cv::Point2f>>> poses2d;
    std::vector<std::vector<CameraParams>> camera_params;

    reader.readPointsFromJSON(file_json_, poses3d_world, poses2d, camera_params);

    for (int idx = 0; idx < camera_params.size(); idx++){
        std::vector<cv::Mat> world2cam_vec{camera_params[idx].size()};
        std::vector<CameraInfo> camera_info{camera_params[idx].size()};
        // Write camera parameters to YAML files
        for (size_t i = 0; i < camera_params[idx].size(); ++i) {
            camera_info[i].setParameters(camera_params[idx][i]);
            world2cam_vec[i] = camera_params[idx][i].extrinsic;
        }
        
        std::vector<cv::Point3f> optimal_3d_points;
        CameraOptimizer camera_optimizer(poses3d_world[idx], world2cam_vec);
        camera_optimizer.optimizer(camera_info, poses2d[idx], optimal_3d_points);
        reader.writePointsToFile(data_ + "optimized_pattern_pts.txt", optimal_3d_points);
    }
    
    //###########################################################
    
}
