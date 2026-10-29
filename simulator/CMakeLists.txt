cmake_minimum_required(VERSION 2.8)

project(${SIMULATOR_EXECUTABLE_NAME})

set(FLAIR_USE_SIMULATOR TRUE)
set(FLAIR_USE_SIMULATOR_GL TRUE)

include("$ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake")

set(SRC_FILES
    src/main.cpp
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/core2-64/simulator_x4.sh.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/core2-64/simulator_x4.sh"
    @ONLY
)

FLAIR_DEMO_HOST_CXX(${PROJECT_NAME} "${SRC_FILES}"
    DEST_DIR ${FLAIR_DEMO_DIRECTORY}
    COPY_RESOURCES
)