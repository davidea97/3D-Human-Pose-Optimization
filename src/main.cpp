#include <iostream>
#include "Optimizer.h"

int main(int argc, char* argv[]) {

    // Check if there are at least two arguments (program name + user argument)
    std::string data_folder, json_file;
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <data_folder> && <json file>" << std::endl;
        return 1;
    }

    if (argc == 3) {
        // Get the user-specified argument from command line
        data_folder = argv[1];
        json_file = argv[2];
        std::cout << "Folder: " << data_folder << std::endl;
        std::cout << "Json file: " << json_file << std::endl;
    }

    if (argc > 3){
        std::cerr << "Too many input arguments!" << std::endl;
        return 1;
    }

    // Start the calibration
    Optimizer optimizer(data_folder, json_file);
    optimizer.optimization();

    return 0;
}

