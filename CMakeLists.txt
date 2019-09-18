cmake_minimum_required (VERSION 3.0)
project(khmot)

set (CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(kalman kalman.cpp)
target_link_libraries(kalman Eigen3::Eigen)
