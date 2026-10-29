cmake_minimum_required(VERSION 2.8)

project(${SIMULATOR_EXECUTABLE_NAME})

set(FLAIR_USE_SIMULATOR TRUE)
set(FLAIR_USE_SIMULATOR_GL TRUE)

include(
    "$ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake"
)

set(SRC_FILES
    src/main.cpp
)

# Host simulation launchers.
set(HOST_LAUNCH_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/generated/core2-64"
)

# Matches the host installation layout observed in this FlAIR environment.
set(HOST_DEMO_INSTALL_DIR
    "$ENV{FLAIR_ROOT}/flair-install/bin/demos/core2-64/${FLAIR_DEMO_DIRECTORY}"
)

file(MAKE_DIRECTORY "${HOST_LAUNCH_DIR}")

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/../uav/launch/x4.sh.in"
    "${HOST_LAUNCH_DIR}/x4.sh"
    @ONLY
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/launch/simulator_x4.sh.in"
    "${HOST_LAUNCH_DIR}/simulator_x4.sh"
    @ONLY
)

FLAIR_DEMO_HOST_CXX(${SIMULATOR_EXECUTABLE_NAME} "${SRC_FILES}"
    DEST_DIR ${FLAIR_DEMO_DIRECTORY}
    COPY_RESOURCES
)

install(
    PROGRAMS
        "${HOST_LAUNCH_DIR}/x4.sh"
        "${HOST_LAUNCH_DIR}/simulator_x4.sh"
    DESTINATION "${HOST_DEMO_INSTALL_DIR}"
)

message(STATUS
    "Host launchers install directory: ${HOST_DEMO_INSTALL_DIR}"
)