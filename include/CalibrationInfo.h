#ifndef CALIBRATIONINFO_H
#define CALIBRATIONINFO_H

#include <string>

// Define a structure to store the data from the YAML file
class CalibrationInfo {
private:
    int number_of_cameras_;
    std::string camera_folder_prefix_;
    int calibration_setup_;
    std::string poses_file_; 

public:
    CalibrationInfo();
    void setNumberOfCams(const int n);
    void setCamFolderPref(const std::string pref);
    void setCalibSetup(const int setup);
    const int getNumberOfCams() const;
    const std::string getCamFolderPref() const;
    const int getCalibSetup() const;
    const std::string getPosesFile() const;
    void printCalibInfo();
};

#endif //CALIBRATIONINFO_H
