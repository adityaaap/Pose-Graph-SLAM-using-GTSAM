# Pose-Graph-SLAM-using-GTSAM
Graph SLAM is implemented on Intel G2O Dataset

# Pose-Graph-SLAM-

# Introduction

This project shows implementation of Graph SLAM algorithm using GTSAM Library. The implementation on 2D and 3D data sets.

Overview:
1.The project shows implementation of Graph SLAM algorithm using GaussNewton and iSAM optimizers to generate optimized trajectories. 
2. Batched and Incremental optimizations are performed on the data sets and the results are compared.

Optimizations:
1. Batch Optimization: A batch solution means when we construct the entire 2D/3D non linear factor graph from the sensor measurements first and solve it at the end altogether. The Optimizer used for this method is Gauss-Newton Optimizer.
2. Incremental Optimizations: Here we incermentally solve the factor graph as and when the measurements come in and loop closures are formed. We use the iSAM2 optimizer for this operation.

The algorithm for incremental solution can be seen here:
<img width="416" alt="image" src="https://github.com/adityaaap/Pose-Graph-SLAM-/assets/112677012/e1dab2c0-df7d-44a8-a637-6bd67e180f98">

# Datasets

1. 2D Data : https://www.dropbox.com/s/vcz8cag7bo0zlaj/input_INTEL_g2o.g2o?dl=0
2. 3D Data : https://www.dropbox.com/s/zu23p8d522qccor/parking-garage.g2o?dl=0

# Getting Started

1. Please first clone and install GTSAM Library to the desired file folder. Detailed instructions can be found here. (https://github.com/borglab/gtsam)

2. Please notice prerequisites: Boost >= 1.58 (Ubuntu: sudo apt-get install libboost-all-dev); CMake>= 3.0 (Ubuntu: sudo apt-get install cmake); A modern compiler, i.e., at least gcc 4.7.3 on Linux.

3. In terminal 
```
cd <path_to_your_desired file folder>
```
4. Then create new build folder
```
mkdir build
```
5. Navigate to build folder
```
cd build
```
6. run cmake
```
cmake ..
```
7. make the file to build folder
```
make -j8
```
8. Install the GTSAM Library
```
sudo make install -j8
```
These steps will install the GTSAM Library in your workspace which can be used further for solving the Pose Grpah SLAM problem.

Additionally this project will also require Eigen Library. The installation instructions can be found here (http://eigen.tuxfamily.org/index.php?title=Main_Page)

# Running the project
This Repo doesn't has the GTSAM library and Eigen installed in it. So when the above steps are complete this project should run.
There 4 combinations of results between 2D,3D and Batched,Incremental solutions. Each file will generate a .g2o result file with the optimized coordinates. This file can be transferrred to Matlab for ease of plotting purposes.

# Results

1. 2D Batched

<img width="422" alt="2D_batched_solution" src="https://github.com/adityaaap/Pose-Graph-SLAM-/assets/112677012/d64f9819-2933-42c8-92d6-68f134e898ca">

2. 2D Incremental

<img width="434" alt="2D_incremental_solution" src="https://github.com/adityaaap/Pose-Graph-SLAM-/assets/112677012/ca9cb061-f6d0-46f7-8709-9dfdbee034bd">

3. 3D Batched

<img width="954" alt="3D_batched_solution" src="https://github.com/adityaaap/Pose-Graph-SLAM-/assets/112677012/bf1b2e0d-8464-4a42-8680-6a6f2f646d9b">

4. 3D Incremental

<img width="959" alt="3D_incremental_solution" src="https://github.com/adityaaap/Pose-Graph-SLAM-/assets/112677012/724acffa-180b-4a43-8820-2b6624bff7cc">

# Acknowledgements

1. GTSAM : https://github.com/borglab/gtsam
2. Datasets : https://lucacarlone.mit.edu/datasets/


