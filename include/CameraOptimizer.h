#ifndef CAMERA_OPTIMIZER_H
#define CAMERA_OPTIMIZER_H

#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include "utils.h"
#include "CameraInfo.h"
#include "minimization.h"
#include "PinholeCameraModel.h"

class CameraOptimizer{
private:
    std::vector< Eigen::Vector3d > pattern_pts_;
    std::vector<Eigen::Vector3d> c2c_r_vecs_;
    std::vector<Eigen::Vector3d> c2c_t_vecs_;

public:
    CameraOptimizer(const std::vector<cv::Point3f> object_points, const std::vector<cv::Mat> c2c_vec);
    void optimizer(const std::vector<CameraInfo> camera_info, std::vector<std::vector<cv::Point2f>> joints_2d, std::vector<cv::Point3f> &optimal_3d_points);
};

#endif //CAMERA_OPTIMIZER_H