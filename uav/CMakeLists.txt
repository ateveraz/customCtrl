PROJECT(customCtrl)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

SET(SRC_FILES
	src/main.cpp
	src/customCtrl.cpp
    src/myCtrl.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR customCtrl
           COPY_RESOURCES
)
