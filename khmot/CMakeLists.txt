cmake_minimum_required (VERSION 3.10)
project(khmot CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Enable debug logging for debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG_LOG")

# By default disable clang-tidy (static code analisys)
if (NOT DEFINED CLANG_TIDY)
  set(CLANG_TIDY "")
endif()

### ROS ###
# Use "cmake -DDISABLE_ROS=TRUE .." to disable ROS
if(NOT DISABLE_ROS)
  find_package(catkin REQUIRED COMPONENTS
    jsk_recognition_msgs
    roscpp
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    khmot_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      jsk_recognition_msgs
      roscpp
      std_msgs
      tf2
      tf2_ros
      tf2_geometry_msgs
      khmot_msgs
  )
endif()
### END ROS ###

add_subdirectory(vendor/eigen)
add_subdirectory(vendor/hungarian-algorithm-cpp)

# testing
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
  src/utils.hpp
)
set_target_properties(khmot PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY}")
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

### ROS ###
if(NOT DISABLE_ROS)
  add_executable(khmot_ros src/ros_tracker.cpp src/ros_tracker_node.cpp)
  set_target_properties(khmot_ros PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY}")
  target_link_libraries(khmot_ros eigen khmot ${catkin_LIBRARIES})
  target_include_directories(khmot_ros PRIVATE src ${catkin_INCLUDE_DIRS})
  install(TARGETS khmot_ros RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
endif()

### END ROS ###
