# CustomCtrl Project

## Overview
The `customCtrl` project provides a framework for developing custom controllers for UAVs based on FlAIR. It includes a skeleton structure for the project, with the `myCtrl` directory containing a customizable controller. In this example, the controller implements a PID-based position controller (with gravity compensation) and an attitude controller. The project also includes utilities for controller saturation and motor constant conversion. The example task becomes from the CircleFollower demo.

## File Structure
- **`uav/src/customCtrl`**: This class contains the skeleton of the project.
- **`uav/src/myCtrl`**: This class defines the customized controller.

## Build Instructions
The project includes a `utils/build.sh` script for building the project. Before running the script, ensure that you modify it to specify the desired build path with the variable `FOLDER_PATH` and the project name with `PROJECT_NAME`.
