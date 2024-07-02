cmake_minimum_required(VERSION 3.16.3)

project(HumanOptimization)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# Find packages
#set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-11.3")
#set(CUDA_CUDA_LIBRARY "/usr/local/cuda-11.3/targets/x86_64-linux/lib/libcudart_static.a")

find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARIES})


find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add an executable target
add_executable(human_optimization src/main.cpp)

# Add a library
add_library(${PROJECT_NAME}_utils 
  src/Optimizer.cpp
  include/Optimizer.h
  src/utils.cpp
  include/utils.h
  src/Reader.cpp
  include/Reader.h
  include/minimization.h
  include/CalibrationInfo.h
  src/CalibrationInfo.cpp
  src/CameraInfo.cpp
  include/CameraInfo.h
  src/CameraOptimizer.cpp
  include/CameraOptimizer.h
  src/PinholeCameraModel.cpp
  include/PinholeCameraModel.h
)

# Link libraries
target_link_libraries(human_optimization
  ${PROJECT_NAME}_utils
  ${OpenCV_LIBS}
  Eigen3::Eigen
  yaml-cpp
  ${CERES_LIBRARIES}
)



