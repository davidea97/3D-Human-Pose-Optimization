#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <iostream>
#include <string>
#include <chrono>

#include "Reader.h"
#include "utils.h"
#include "CameraOptimizer.h"
#include "CameraInfo.h"

#include <Eigen/Dense>

class Optimizer{
private:
    std::string data_;
    std::string file_json_;

public:
    Optimizer(std::string data_folder, std::string json_file){data_ = data_folder; file_json_ = json_file;};
    void optimization();
};


#endif //OPTIMIZER_H
