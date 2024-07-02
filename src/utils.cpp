#include <fstream>
#include "utils.h"

namespace fs = std::filesystem;

// Function to calculate translation error
double computeTranslationError(const cv::Mat& A, const cv::Mat& B) {
    cv::Vec3f translationA = A(cv::Range(0, 3), cv::Range(3, 4));
    cv::Vec3f translationB = B(cv::Range(0, 3), cv::Range(3, 4));
    return cv::norm(translationA - translationB);
}

// Function to convert cv::Mat to Eigen::Matrix3f
Eigen::Matrix3f cvMatToEigenMatrix(const cv::Mat& mat) {
    Eigen::Matrix3f eigenMat;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            eigenMat(i, j) = mat.at<float>(i, j);
    return eigenMat;
}

// Function to calculate rotation error in radians
double computeRotationError(const cv::Mat& A, const cv::Mat& B) {
    cv::Mat R_A = A(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat R_B = B(cv::Range(0, 3), cv::Range(0, 3));
    
    Eigen::Matrix3f eigen_R_A = cvMatToEigenMatrix(R_A);
    Eigen::Matrix3f eigen_R_B = cvMatToEigenMatrix(R_B);
    
    Eigen::Matrix3f R_error_matrix = eigen_R_A * eigen_R_B.transpose();
    Eigen::AngleAxisf angle_axis(R_error_matrix);
    
    return std::abs(angle_axis.angle());
}

bool isFolderNotEmpty(const std::string& folder_path) {
    if (!fs::exists(folder_path)) {
        // The folder does not exist
        std::cerr << "The folder does not exist!" << std::endl;
        return false;
    }

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        // If there is at least one entry in the folder, it's not empty
        return true;
    }

    // The folder is empty
    std::cerr << "The folder is empty!" << std::endl;
    return false;
}


void checkData(std::string data_folder, std::string prefix, int number_of_cameras){
    int folder_count = 0;
    for (const auto& entry: fs::directory_iterator(data_folder)){
        if (entry.path().filename().string().find(prefix) == 0){
            folder_count ++;
        }
    }

    // Stop the program if the number of selected cameras is different from the number of provided folders
    if (number_of_cameras != folder_count) {
        std::cerr << "The number of selected cameras does not coincide with the number of provided folders!" << std::endl;
        exit(EXIT_FAILURE);
    }
}


void createFolder(std::string folder_name){
    if (!fs::is_directory(folder_name)) {
        try {
            fs::create_directories(folder_name);
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error creating the folder: " << e.what() << std::endl;
        }
    }
}

int linesNumber(const std::string file_path){
    std::vector<std::string> existing_data;
    std::ifstream input_file(file_path);
    if (input_file.is_open()) {
        std::string line;
        while (std::getline(input_file, line)) {
            existing_data.push_back(line);
        }
        input_file.close();
    }

    return existing_data.size();
}



std::string getStringAfterLastSlash(const std::string& input) {
    // Trova la posizione dell'ultima occorrenza del carattere '/'
    size_t pos = input.find_last_of('/');

    // Se '/' è stato trovato nella stringa
    if (pos != std::string::npos) {
        // Estrai e restituisce la sottostringa che segue l'ultima '/'
        return input.substr(pos + 1);
    } else {
        // Se '/' non è presente, restituisce una stringa vuota o un messaggio
        return "Nessun carattere '/' trovato nella stringa.";
    }
}





int countImagesInFolder(const std::string& path) {
    int count = 0;
    for (const auto& entry : fs::directory_iterator(path)) {
        if (entry.is_regular_file()) {
            auto ext = entry.path().extension().string();
            
            // Convert extension to lowercase to make the comparison case-insensitive
            std::transform(ext.begin(), ext.end(), ext.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
                count++;
            }
        }
    }
    return count;
}

// Function to extract the numeric part from the filename
int extract_number(const std::string& filename) {
    std::regex re(".*_(\\d+)\\.json");
    std::smatch match;
    if (std::regex_search(filename, match, re)) {
        return std::stoi(match[1].str());
    }
    return -1; // or throw an error if format is guaranteed
}
