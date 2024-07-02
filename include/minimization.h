#ifndef REPROJECTION_ERROR_MINIMIZATION_H
#define REPROJECTION_ERROR_MINIMIZATION_H

#include "Eigen/Core"
#include "ceres/ceres.h"
#include <ceres/rotation.h>
#include "PinholeCameraModel.h"

struct MinimizeReprojectionError
{
    MinimizeReprojectionError( const PinholeCameraModel &cam_model,
                            const Eigen::Vector3d &c2c_r_vecs_,
                            const Eigen::Vector3d &c2c_t_vecs_,
                            const Eigen::Vector2d &observed_pt ) :
            cam_model(cam_model),
            c2c_r_vecs_(c2c_r_vecs_),
            c2c_t_vecs_(c2c_t_vecs_),
            observed_pt( observed_pt) {}

    //template <typename T>
    bool operator()(//const double* const weights,
            const double* const joints_3d,
            double* residuals) const
    {
        double ptn_pt[3] = {double(joints_3d[0]), double(joints_3d[1]), double(joints_3d[2])},
                cam_pt[3];

        double c2c_r[3] = {double(c2c_r_vecs_(0)), double(c2c_r_vecs_(1)), double(c2c_r_vecs_(2))},
               c2c_t[3] = {double(c2c_t_vecs_(0)), double(c2c_t_vecs_(1)), double(c2c_t_vecs_(2))};

        // Apply the current board2ee transformation
        ceres::AngleAxisRotatePoint(c2c_r, ptn_pt, cam_pt);

        cam_pt[0] += c2c_t[0];
        cam_pt[1] += c2c_t[1];
        cam_pt[2] += c2c_t[2];

        // Projection.
        double proj_pt[2];
        cam_model.project(cam_pt, proj_pt);

        residuals[0] = (proj_pt[0] - double(observed_pt(0)));
        residuals[1] = (proj_pt[1] - double(observed_pt(1)));
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &c2c_r_vecs_,
                                        const Eigen::Vector3d &c2c_t_vecs_,
                                        const Eigen::Vector2d &observed_pt )
    {
        return (new ceres::NumericDiffCostFunction<MinimizeReprojectionError, ceres::RIDDERS,  2, 3>(
                new MinimizeReprojectionError( cam_model, c2c_r_vecs_, c2c_t_vecs_, observed_pt )));
    }

    const PinholeCameraModel &cam_model;
    Eigen::Vector3d c2c_r_vecs_;
    Eigen::Vector3d c2c_t_vecs_;
    Eigen::Vector2d observed_pt;
};


#endif //REPROJECTION_ERROR_MINIMIZATION_H
