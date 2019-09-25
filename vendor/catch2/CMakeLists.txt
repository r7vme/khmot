add_library(catch2 INTERFACE)
set_target_properties(catch2
    PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}"
)
