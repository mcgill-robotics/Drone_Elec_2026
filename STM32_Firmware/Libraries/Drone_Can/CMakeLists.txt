cmake_minimum_required(VERSION 3.22)

# Collect all sources
file(GLOB_RECURSE LIB_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/can_node/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Canard/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/UAVCAN_DSDL/src/*.c"
)

# Define the static library target
add_library(Drone_Can STATIC
    ${LIB_SOURCES}
)

# Expose include paths to this library and anything that links it
target_include_directories(Drone_Can PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/can_node/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Canard
    ${CMAKE_CURRENT_SOURCE_DIR}/UAVCAN_DSDL/include
)