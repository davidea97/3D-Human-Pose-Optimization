#include "CameraOptimizer.h"

CameraOptimizer::CameraOptimizer(const std::vector<cv::Point3f> object_points, const std::vector<cv::Mat> c2c_vec) {
    

    cv::Mat_<float> r_vec_c2c, t_vec_c2c;
    c2c_r_vecs_.resize(c2c_vec.size());
    c2c_t_vecs_.resize(c2c_vec.size());

    // Save tvec and rvec for each robot path_pose
    for (int i = 0; i < c2c_vec.size(); i++){
        cv::Mat cam_pose_temp = c2c_vec[i];
        transfMat2Exp<float>(cam_pose_temp, r_vec_c2c, t_vec_c2c);
        for( int j = 0; j < 3; j++ ){
            c2c_r_vecs_[i](j) = r_vec_c2c(j);
            c2c_t_vecs_[i](j) = t_vec_c2c(j);
        }
    }


    // Object point initialization
    pattern_pts_.reserve(object_points.size());

    for( auto &cp : object_points )
        pattern_pts_.emplace_back( cp.x, cp.y, cp.z );
}


void CameraOptimizer::optimizer(const std::vector<CameraInfo> camera_info, std::vector<std::vector<cv::Point2f>> joints_2d, std::vector<cv::Point3f> &optimal_3d_points) {
    double observed_pt_data[2];
    Eigen::Map<Eigen::Vector2d> observed_pt(observed_pt_data);

    // Ceres problem
    ceres::Solver::Options options;

    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.preconditioner_type = ceres::JACOBI;

    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 18;
    options.max_num_iterations = 3000;
    options.gradient_tolerance = 1e-5;
    options.function_tolerance = std::numeric_limits<double>::epsilon();
    options.parameter_tolerance = 1e-8;
    double humer_a_scale = 1.0;
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(humer_a_scale);

    ceres::Problem problem;

    std::vector<PinholeCameraModel> cam_model_vec;
    for (int i = 0; i < camera_info.size(); i++){
        PinholeCameraModel camera_model(camera_info[i]);
        cam_model_vec.push_back(camera_model);
    }

    
    for (int i = 0; i < camera_info.size(); i++){
        auto images_points = joints_2d[i];
        for( int k = 0; k < pattern_pts_.size(); k++ ) {
            observed_pt_data[0] = images_points[k].x;
            observed_pt_data[1] = images_points[k].y;

            ceres::CostFunction *cost_function = MinimizeReprojectionError::Create(cam_model_vec[i], c2c_r_vecs_[i], c2c_t_vecs_[i], observed_pt);

            problem.AddResidualBlock(cost_function, loss_function, pattern_pts_[k].data());
        }
    }

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // Fill optimal_points with the optimized pattern_pts_
    optimal_3d_points.reserve(pattern_pts_.size());
    for (const auto &pt : pattern_pts_) {
        optimal_3d_points.emplace_back(pt.x(), pt.y(), pt.z());
    }
}