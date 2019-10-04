cmake_minimum_required (VERSION 3.5)
project(khmot CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

add_subdirectory(vendor/eigen)
add_subdirectory(vendor/hungarian-algorithm-cpp)

# testing
set(DISABLE_TESTS FALSE)
if(NOT DISABLE_TESTS)
  add_subdirectory(vendor/catch2)
  enable_testing()
  include(Catch)
endif()

add_library(khmot STATIC
  src/tracker.hpp
  src/tracker.cpp
  src/kalman.hpp
  src/kalman.cpp
)
target_link_libraries(khmot eigen hungarian)
target_compile_options(khmot PRIVATE -Werror -Wall -Wextra)

if(NOT DISABLE_TESTS)
  add_executable(test_khmot
    src/kalman_test.cpp
    src/tracker_test.cpp
  )
  target_link_libraries(test_khmot catch2 khmot)
  target_compile_options(test_khmot PRIVATE -Werror -Wall -Wextra)
  catch_discover_tests(test_khmot)
endif()

add_executable(main main.cpp)
target_link_libraries(main eigen khmot)
target_include_directories(main PRIVATE src)
