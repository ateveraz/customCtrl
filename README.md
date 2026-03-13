# CustomCtrl Project

## Overview
The `customCtrl` project provides a framework for developing custom controllers for UAVs based on FlAIR. It includes a skeleton structure for the project, with `myCtrl` containing a customizable controller. In this example, the controller implements a PD-based position controller (with gravity compensation) and an PD attitude controller. The project also includes utilities for controller saturation and motor constant conversion. The example task is derived from the CircleFollower demo.

## File structure
- **`uav/src/customCtrl`**: This class contains the skeleton of the project.
- **`uav/src/myCtrl`**: This class defines the customized controller.

## About Building the Project
You can build this template by following the instructions in the section below (strongly recommended) or by following the instructions in the FlAIR Wiki: https://gitlab.utc.fr/uav-hds/flair/flair-src/-/wikis/build-system. By default, the project is named `customCtrl`.

If you want to rename it, you can clone this folder using the instructions in the FlAIR Wiki for cloning a demo: https://gitlab.utc.fr/uav-hds/flair/flair-src/-/wikis/demos/clone. This allows you to clone the project and set a custom source directory name.

## VS Code & CLion Setup
This project includes a setup script `setup4vscode.sh` to generate configurations (CMakePresets, IntelliSense, etc.) for VS Code and CLion.

### Usage
Run the script from the project root:
```bash
chmod +x setup4vscode.sh
./setup4vscode.sh [options]
```

### Options
- **`-f <path>`**: (Optional) Specify a relative path inside `flair-build` for build artifacts. This allows you to control the exact structure of your build output directory.
  - **Default**: `mysrc/${sourceDirName}`
  - **Example**: 
    ```bash
    ./setup4vscode.sh -f custom_path/my_test
    ```
    This will configure the build output to be `flair-build/custom_path/my_test`.