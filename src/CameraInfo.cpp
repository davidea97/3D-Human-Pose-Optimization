#include "CameraInfo.h"
#include "utils.h"

CameraInfo::CameraInfo(){
}

float CameraInfo::getFx() const{
    return this->fx_;
}

void CameraInfo::setFx(float new_fx){
    this->fx_ = new_fx;
}

float CameraInfo::getFy() const{
    return this->fy_;
}

void CameraInfo::setFy(float new_fy){
    this->fy_ = new_fy;
}

float CameraInfo::getCx() const {
    return this->cx_;
}

void CameraInfo::setCx(float new_cx) {
    this->cx_ = new_cx;
}

float CameraInfo::getCy() const {
    return this->cy_;
}

void CameraInfo::setCy(float new_cy){
    this->cy_ = new_cy;
}

float CameraInfo::getDistK0() const {
    return this->dist_k0_;
}

float CameraInfo::getDistK1() const {
    return this->dist_k1_;
}

float CameraInfo::getDistPx() const {
    return this->dist_px_;
}

float CameraInfo::getDistPy() const {
    return this->dist_py_;
}

float CameraInfo::getDistK2() const {
    return this->dist_k2_;
}

float CameraInfo::getDistK3() const {
    return this->dist_k3_;
}

float CameraInfo::getDistK4() const {
    return this->dist_k4_;
}

float CameraInfo::getDistK5() const {
    return this->dist_k5_;
}

int CameraInfo::getImageWidth() const {
    return this->img_width_;
}

int CameraInfo::getImageHeight() const {
    return this->img_height_;
}

cv::Mat CameraInfo::getCameraMatrix() const {
    cv::Mat camera_matrix = (cv::Mat_<float>(3,3) << this->fx_, 0, this->cx_, 0, this->fy_, this->cy_, 0, 0, 1);
    return camera_matrix;
}

void CameraInfo::setCameraMatrix(cv::Mat new_camera_matrix) {
    this->fx_ = new_camera_matrix.at<float>(0,0);
    this->fy_ = new_camera_matrix.at<float>(1,1);
    this->cx_ = new_camera_matrix.at<float>(0,2);
    this->cy_ = new_camera_matrix.at<float>(1,2);
}

cv::Mat CameraInfo::getDistCoeff() const {
    cv::Mat distortion_coeff = (cv::Mat_<float>(1,8) << this->dist_k0_, this->dist_k1_, this->dist_px_, this->dist_py_,
                                                        this->dist_k2_, this->dist_k3_, this->dist_k4_, this->dist_k5_);
    return distortion_coeff;
}

void CameraInfo::setDistCoeff(cv::Mat new_dist_coeff){
    this->dist_k0_ = new_dist_coeff.at<float>(0,0);
    this->dist_k1_ = new_dist_coeff.at<float>(0,1);
    this->dist_k2_ = new_dist_coeff.at<float>(0,2);
    this->dist_k3_ = new_dist_coeff.at<float>(0,3);
    this->dist_k4_ = new_dist_coeff.at<float>(0,4);
    this->dist_k5_ = new_dist_coeff.at<float>(0,5);
    this->dist_px_ = new_dist_coeff.at<float>(0,6);
    this->dist_py_ = new_dist_coeff.at<float>(0,7);
}

bool CameraInfo::setParameters(const CameraParams& camera_params){

    try {
        this->fx_ = camera_params.intrinsic.at<double>(0,0);
        this->fy_ = camera_params.intrinsic.at<double>(1,1);
        this->cx_ = camera_params.intrinsic.at<double>(0,2);
        this->cy_ = camera_params.intrinsic.at<double>(1,2);

        this->has_dist_coeff_ = 1;

        this->dist_k0_ = camera_params.dist.at<double>(0);
        this->dist_k1_ = camera_params.dist.at<double>(1);
        this->dist_px_ = camera_params.dist.at<double>(2);
        this->dist_py_ = camera_params.dist.at<double>(3);
        this->dist_k2_ = camera_params.dist.at<double>(4);
        this->dist_k3_ = 0;
        this->dist_k4_ = 0;
        this->dist_k5_ = 0;

        this->img_width_ = 1000;
        this->img_height_ = 1002;

        return true;
    } catch (const YAML::Exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

