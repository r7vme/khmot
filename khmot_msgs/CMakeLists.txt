cmake_minimum_required (VERSION 3.5)
project(khmot_msgs CXX)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  BoundingBoxWithCovariance.msg
  BoundingBoxWithCovarianceArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    std_msgs
)
