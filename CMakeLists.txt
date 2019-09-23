cmake_minimum_required (VERSION 3.5)
project(khmot CXX)

add_subdirectory(vendor/eigen)
# testing
#add_subdirectory(vendor/catch2)

add_library(kalman STATIC kalman.hpp kalman.cpp)
target_link_libraries(kalman eigen)

add_executable(main main.cpp)
target_link_libraries(main kalman)
