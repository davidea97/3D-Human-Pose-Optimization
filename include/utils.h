#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <filesystem>
#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <random>
#include <opencv2/core/affine.hpp>
#include <cmath>
#include <opencv2/features2d.hpp>
#include <algorithm>
#include <regex>

#include "CalibrationInfo.h"

struct Transformation {
    double tx, ty, tz; // Traslazione: x, y, z
    double rx, ry, rz; // Rotazione: x, y, z
};

struct LinearRegressionResult {
    double slope;
    double intercept;
};

// Function to calculate translation error
double computeTranslationError(const cv::Mat& A, const cv::Mat& B);
Eigen::Matrix3f cvMatToEigenMatrix(const cv::Mat& mat);
// Function to calculate rotation error in radians
double computeRotationError(const cv::Mat& A, const cv::Mat& B);

bool isFolderNotEmpty(const std::string& folder_path);
void checkData(std::string data_folder, std::string prefix, int number_of_cameras);
void createFolder(std::string folder_name);
int linesNumber(const std::string file_path);
std::string getStringAfterLastSlash(const std::string& input);
int countImagesInFolder(const std::string& path);
int extract_number(const std::string& filename);

template <typename _T>
void getRotoTras(cv::Mat rotation, cv::Mat translation, cv::Mat& G){
    G = (cv::Mat_<_T>(4,4) << rotation.at<_T>(0,0), rotation.at<_T>(0,1), rotation.at<_T>(0,2), translation.at<_T>(0),
            rotation.at<_T>(1,0), rotation.at<_T>(1,1), rotation.at<_T>(1,2), translation.at<_T>(1),
            rotation.at<_T>(2,0), rotation.at<_T>(2,1), rotation.at<_T>(2,2), translation.at<_T>(2),
            0.0, 0.0, 0.0, 1.0);
}

template <typename _T>
void getRoto(cv::Mat G, cv::Mat& rotation){
    rotation = (cv::Mat_<_T>(3,3) << G.at<_T>(0,0), G.at<_T>(0,1), G.at<_T>(0,2),
            G.at<_T>(1,0), G.at<_T>(1,1), G.at<_T>(1,2),
            G.at<_T>(2,0), G.at<_T>(2,1), G.at<_T>(2,2));
}

template <typename _T>
void getTras(cv::Mat G, cv::Mat& translation){
    translation = (cv::Mat_<_T>(1,3) << G.at<_T>(0,3), G.at<_T>(1,3), G.at<_T>(2,3));
}


template <typename _T>
cv::Mat rotationMatrixToEulerAngles(const cv::Mat &R) {
    assert(R.rows == 3 && R.cols == 3);

    _T sy = sqrt(R.at<_T>(0,0) * R.at<_T>(0,0) +  R.at<_T>(1,0) * R.at<_T>(1,0));

    bool singular = sy < 1e-6; // Se sy è vicino a zero, la direzione dell'asse z è vicino a singolarità

    _T x, y, z;
    if (!singular) {
        x = atan2(R.at<_T>(2,1), R.at<_T>(2,2));
        y = atan2(-R.at<_T>(2,0), sy);
        z = atan2(R.at<_T>(1,0), R.at<_T>(0,0));
    } else {
        x = atan2(-R.at<_T>(1,2), R.at<_T>(1,1));
        y = atan2(-R.at<_T>(2,0), sy);
        z = 0;
    }
    cv::Mat euler_mat = (cv::Mat_<_T>(1,3) << 0,0,0);
    euler_mat.at<_T>(0,0) = x;
    euler_mat.at<_T>(0,1) = y;
    euler_mat.at<_T>(0,2) = z;
    return euler_mat;
}

template <typename _T>
cv::Mat eulerAnglesToRotationMatrix(const cv::Mat& rvec) {

    cv::Mat R_x = (cv::Mat_<_T>(3, 3) <<
                                          1, 0, 0,
            0, cos(rvec.at<_T>(0)), -sin(rvec.at<_T>(0)),
            0, sin(rvec.at<_T>(0)), cos(rvec.at<_T>(0)));

    cv::Mat R_y = (cv::Mat_<_T>(3, 3) <<
                                          cos(rvec.at<_T>(1)), 0, sin(rvec.at<_T>(1)),
            0, 1, 0,
            -sin(rvec.at<_T>(1)), 0, cos(rvec.at<_T>(1)));

    cv::Mat R_z = (cv::Mat_<_T>(3, 3) <<
                                          cos(rvec.at<_T>(2)), -sin(rvec.at<_T>(2)), 0,
            sin(rvec.at<_T>(2)), cos(rvec.at<_T>(2)), 0,
            0, 0, 1);

    cv::Mat R = R_z * R_y * R_x;

    return R;
}

template <typename _T>
cv::Mat eulerAnglesToRotationMatrix(const std::vector<_T>& theta) {
    // Assicurati che il vettore theta abbia tre elementi
    assert(theta.size() == 6);

    // Calcola le matrici di rotazione intorno a ciascun asse
    cv::Mat R_x = (cv::Mat_<_T>(3, 3) <<
                                          1, 0, 0,
            0, cos(theta[0]), -sin(theta[0]),
            0, sin(theta[0]), cos(theta[0]));

    cv::Mat R_y = (cv::Mat_<_T>(3, 3) <<
                                          cos(theta[1]), 0, sin(theta[1]),
            0, 1, 0,
            -sin(theta[1]), 0, cos(theta[1]));

    cv::Mat R_z = (cv::Mat_<_T>(3, 3) <<
                                          cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]), cos(theta[2]), 0,
            0, 0, 1);

    // Combina le rotazioni in una singola matrice di rotazione
    cv::Mat R = R_z * R_y * R_x;
    cv::Mat t = (cv::Mat_<_T>(1,3) << theta[3], theta[4], theta[5]);
    cv::Mat G;
    getRotoTras<_T>(R,t,G);

    return G;
}

template <typename _T>
void addGaussianNoiseToElement(cv::Mat &matrix, int row, int col, _T mean, _T stddev) {
    // Create a single element matrix for the noise
    cv::Mat noise = cv::Mat::zeros(1, 1, matrix.type());
    cv::randn(noise, mean, stddev);

    // Add the noise to the specific element in the matrix
    matrix.at<_T>(row, col) += noise.at<_T>(0, 0);

}

template <typename _T>
cv::Mat addGaussianNoise(cv::Mat original_pose, _T tx, _T ty, _T tz, _T rx, _T ry, _T rz){
    cv::Mat rotation, tvec;
    getRoto<_T>(original_pose, rotation);
    getTras<_T>(original_pose, tvec);

    cv::Mat rvec, rotation_noisy;
    rvec = rotationMatrixToEulerAngles<_T>(rotation);

    // Define mean and standard deviation for each component
    _T means[3] = {0.0, 0.0, 0.0};
    _T stddevs_r[3] = {rx, ry, rz}; // Different stddev for each component
    _T stddevs_t[3] = {tx, ty, tz};

    // Add Gaussian noise to each component of translation and rotation matrices
    for (int k = 0; k < 3; ++k) {
        addGaussianNoiseToElement<_T>(tvec, 0, k, means[k], stddevs_t[k]);
        addGaussianNoiseToElement<_T>(rvec, 0, k, means[k], stddevs_r[k]);
    }

    //cv::Rodrigues(rvec, rotation_noisy);
    rotation_noisy = eulerAnglesToRotationMatrix<_T>(rvec);

    cv::Mat noisy_pose;
    getRotoTras<_T>(rotation_noisy, tvec, noisy_pose);

    return noisy_pose;
}






template <typename _T>
_T getAverage(std::vector<_T> vec){
    _T sum = 0;
    for (int i = 0; i < vec.size(); i++){
        sum += vec[i];
    }
    _T average = sum/vec.size();

    return average;
}


template <typename _T, int _ROWS, int _COLS>
void openCv2Eigen( const cv::Mat_<_T> &cv_mat,
                   Eigen::Matrix<_T, _ROWS, _COLS> &eigen_mat )
{
    for(int r = 0; r < _ROWS; r++)
        for(int c = 0; c < _COLS; c++)
            eigen_mat(r,c) = cv_mat(r,c);
}

template <typename _T, int _ROWS, int _COLS>
void eigen2openCv( const Eigen::Matrix<_T, _ROWS, _COLS> &eigen_mat,
                   cv::Mat_<_T> &cv_mat )
{
    cv_mat = cv::Mat_<_T>(_ROWS,_COLS);

    for(int r = 0; r < _ROWS; r++)
        for(int c = 0; c < _COLS; c++)
            cv_mat(r,c) = eigen_mat(r,c);
}

template <typename _T>
void rotMat2AngleAxis( const cv::Mat &r_mat, cv::Mat &r_vec )
{
    cv::Mat_<_T>tmp_r_mat(r_mat);
    cv::Rodrigues(tmp_r_mat, r_vec );
}

template <typename _T>
void rotMat2AngleAxis( const cv::Mat &r_mat, cv::Vec<_T, 3> &r_vec )
{
    cv::Mat_<_T>tmp_r_mat(r_mat), tmp_r_vec;
    cv::Rodrigues(tmp_r_mat, tmp_r_vec );
    r_vec[0] = tmp_r_vec(0); r_vec[1] = tmp_r_vec(1); r_vec[2] = tmp_r_vec(2);
}

template <typename _T>
void rotMat2AngleAxis( const Eigen::Matrix<_T, 3, 3> &r_mat, cv::Mat &r_vec )
{
    cv::Mat_<_T> tmp_r_mat;
    eigen2openCv<_T, 3, 3>(r_mat, tmp_r_mat);
    cv::Rodrigues(tmp_r_mat, r_vec );
}

template <typename _T>
void rotMat2AngleAxis( const Eigen::Matrix<_T, 3, 3> &r_mat, cv::Vec<_T, 3> &r_vec )
{
    cv::Mat_<_T> tmp_r_vec;
    rotMat2AngleAxis<_T>( r_mat, tmp_r_vec );
    r_vec[0] = tmp_r_vec(0); r_vec[1] = tmp_r_vec(1); r_vec[2] = tmp_r_vec(2);
}

template <typename _T>
void angleAxis2RotMat( const cv::Mat &r_vec, cv::Mat &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec);
    cv::Rodrigues(tmp_r_vec, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Vec<_T, 3> &r_vec, cv::Mat &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec);
    cv::Rodrigues(tmp_r_vec, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Mat &r_vec, Eigen::Matrix<_T, 3, 3> &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec), tmp_r_mat;
    cv::Rodrigues(tmp_r_vec, tmp_r_mat );
    openCv2Eigen<_T, 3, 3>( tmp_r_mat, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Vec<_T, 3> &r_vec, Eigen::Matrix<_T, 3, 3> &r_mat)
{
    cv::Mat_<_T>tmp_r_vec(r_vec), tmp_r_mat;
    cv::Rodrigues(tmp_r_vec, tmp_r_mat );
    openCv2Eigen<_T, 3, 3>( tmp_r_mat, r_mat );
}

template <typename _T>
void exp2TransfMat( const cv::Mat &r_vec, const cv::Mat &t_vec, cv::Mat &g_mat )
{
    cv::Mat_<_T> tmp_t_vec(t_vec), r_mat;
    angleAxis2RotMat<_T>( r_vec, r_mat );

    g_mat = (cv::Mat_< _T >(4, 4)
            << r_mat(0,0), r_mat(0,1), r_mat(0,2), tmp_t_vec(0,0),
            r_mat(1,0), r_mat(1,1), r_mat(1,2), tmp_t_vec(1,0),
            r_mat(2,0), r_mat(2,1), r_mat(2,2), tmp_t_vec(2,0),
            0,          0,          0,          1);

}
template <typename _T>
void transfMat2Exp( const cv::Mat &g_mat, cv::Mat &r_vec, cv::Mat &t_vec )
{
    cv::Mat_<_T> tmp_g_mat(g_mat);
    cv::Mat_< _T > r_mat = (cv::Mat_< _T >(3, 3)
            << tmp_g_mat(0,0), tmp_g_mat(0,1), tmp_g_mat(0,2),
            tmp_g_mat(1,0), tmp_g_mat(1,1), tmp_g_mat(1,2),
            tmp_g_mat(2,0), tmp_g_mat(2,1), tmp_g_mat(2,2));
    rotMat2AngleAxis<_T>(r_mat, r_vec);
    t_vec = (cv::Mat_< _T >(3, 1)<<tmp_g_mat(0,3), tmp_g_mat(1,3), tmp_g_mat(2,3));
}


struct OptimizationData {
    std::vector<std::pair<cv::Mat, cv::Mat>> transformPairs; // Coppie di matrici A e B
    Eigen::Matrix4d X_fixed; // Parte fissa di X, esclusa tz
    Eigen::Matrix4d Z_fixed; // Parte fissa di Z, esclusa tz
};



#endif //UTILS_H
