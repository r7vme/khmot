cmake_minimum_required (VERSION 3.5)
project(khmot CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

add_subdirectory(vendor/eigen)

# testing
add_subdirectory(vendor/catch2)
enable_testing()
include(Catch)

add_library(kalman STATIC src/kalman.hpp src/kalman.cpp)
target_link_libraries(kalman eigen)

add_executable(test_kalman src/kalman_test.cpp)
target_link_libraries(test_kalman kalman catch2)
catch_discover_tests(test_kalman)

#add_executable(main main.cpp)
#target_link_libraries(main kalman)
