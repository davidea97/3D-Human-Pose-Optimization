# 3D-Human-Pose-Optimization
Optimization procedure to refine the 3D joints of humans by minimizing the reprojection error on multiple views

## Compile the project
```bash
# Clone the repository
git clone https://github.com/davidea97/3D-Human-Pose-Optimization.git

# Navigate to the project directory
cd 3D-Human-Pose-Optimization

# Navigate to the project directory and compile
mkdir build 
cd build
cmake ..
make
```

## Run the optimizer
```bash
# Run the code 
./human_optimization <test_folder> <json_file_with_data>


