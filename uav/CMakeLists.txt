cmake_minimum_required(VERSION 2.8)

project(${UAV_EXECUTABLE_NAME})

set(FLAIR_USE_FILTER TRUE)
set(FLAIR_USE_SENSOR_ACTUATOR TRUE)
set(FLAIR_USE_META TRUE)
set(FLAIR_USE_VRPN TRUE)

include("$ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake")

set(SRC_FILES
    src/main.cpp
    src/UavApplication.cpp
    src/CustomControl.cpp
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/core2-64/x4.sh.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/core2-64/x4.sh"
    @ONLY
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
    DEST_DIR ${FLAIR_DEMO_DIRECTORY}
    COPY_RESOURCES
)