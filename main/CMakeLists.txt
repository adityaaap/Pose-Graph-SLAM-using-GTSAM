cmake_minimum_required(VERSION 2.8.11)
# project (HW)

# add_library (HW hello.cpp)
# target_link_libraries(HW gtsam)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

add_executable(HW_Q1_2D HW_Q1_2D.cpp)
add_executable(HW_Q2_3D HW_Q2_3D.cpp)

target_link_libraries(HW_Q1_2D gtsam)
target_link_libraries(HW_Q2_3D gtsam)

set_target_properties(HW_Q1_2D PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
set_target_properties(HW_Q2_3D PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)


