#include <iostream>
#include "CalibrationInfo.h"

CalibrationInfo::CalibrationInfo() {}

void CalibrationInfo::setNumberOfCams(const int n){
    this->number_of_cameras_ = n;
}

void CalibrationInfo::setCamFolderPref(const std::string pref){
    this->camera_folder_prefix_ = pref;
}

void CalibrationInfo::setCalibSetup(const int setup){
    this->calibration_setup_ = setup;
}

const int CalibrationInfo::getNumberOfCams() const {
    return this->number_of_cameras_;
}

const std::string CalibrationInfo::getCamFolderPref() const {
    return this->camera_folder_prefix_;
}

const int CalibrationInfo::getCalibSetup() const {
    return this->calibration_setup_;
}

const std::string CalibrationInfo::getPosesFile() const {
    return this->poses_file_;
}


void CalibrationInfo::printCalibInfo() {
    std::cout << "------------- Calibration parameters -------------" << std::endl;
    std::cout << "Calibration type: " ;
    switch (this->calibration_setup_){
        case 0:
            std::cout << "Eye-in-hand" << std::endl;
            break;
        case 1:
            std::cout << "Eye-on-base" << std::endl;
            break;
    }
    std::cout << "Number of cameras: " << this->number_of_cameras_ << std::endl;
}