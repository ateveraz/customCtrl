# !/bin/bash

# This script is used to create a new project in the build directory.
# Further details: https://gitlab.utc.fr/uav-hds/flair/flair-src/-/wikis/build-system

# Define the folder and project name
FOLDER_PATH='templates'
PROJECT_NAME='customCtrl'
SRC_PATH=$(pwd)

echo $SRC_PATH

# Create the folder into the build directory
cd $FLAIR_ROOT/flair-build/
mkdir -p $FOLDER_PATH
cd $FOLDER_PATH
mkdir -p $PROJECT_NAME

# # Goto the folder
cd $PROJECT_NAME
$FLAIR_ROOT/flair-src/scripts/cmake_codelite_outofsource.sh $SRC_PATH

# echo $(pwd)
cd build
make clean
make install -j`nproc`

# # Change to executable mode
chmod +x $FLAIR_ROOT/flair-install/bin/demos/core2-64/$PROJECT_NAME/*.sh