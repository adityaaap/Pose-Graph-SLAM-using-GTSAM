cmake_minimum_required(VERSION 2.8.11)
# project (HW)

# add_library (HW hello.cpp)
# target_link_libraries(HW gtsam)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

add_executable(2D_Graph_SLAM 2D_Graph_SLAM.cpp)
add_executable(3D_Graph_SLAM 3D_Graph_SLAM.cpp)

target_link_libraries(2D_Graph_SLAM gtsam)
target_link_libraries(3D_Graph_SLAM gtsam)

set_target_properties(2D_Graph_SLAM PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
set_target_properties(3D_Graph_SLAM PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)


