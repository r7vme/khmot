cmake_minimum_required (VERSION 3.5)
project(khmot_msgs CXX)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Observation.msg
  ObservationsArray.msg
  Track.msg
  TracksArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
