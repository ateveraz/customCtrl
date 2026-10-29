cmake_minimum_required(VERSION 2.8)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ProjectConfig.cmake")

project(${UAV_PROJECT_NAME})

if("$ENV{FLAIR_ROOT}" STREQUAL "")
    message(FATAL_ERROR
        "FLAIR_ROOT is not defined. Load the FlAIR/SDK environment first.")
endif()

include("$ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake")

add_subdirectory(simulator)
add_subdirectory(uav)