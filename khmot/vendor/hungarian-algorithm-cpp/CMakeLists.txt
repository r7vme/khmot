add_library(hungarian STATIC Hungarian.h Hungarian.cpp)
target_include_directories(hungarian
  INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}"
)
