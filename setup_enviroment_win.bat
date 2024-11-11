@echo off
REM Initialize submodules
git submodule update --init --remote --recursive

REM Set base and destination directories
set "BASE_DIR=%cd%"
set "DEST_DIR=%BASE_DIR%\ros2_ws\src\sign_recognition_node\sign_recognition_node\Distance_Sign_Recognition"

REM Export the PYTHONPATH
set "PYTHONPATH=%PYTHONPATH%;%DEST_DIR%"
set "PYTHONPATH=%PYTHONPATH%;%DEST_DIR%\image_processing"
set "PYTHONPATH=%PYTHONPATH%;%DEST_DIR%\utils"

REM Navigate to the Models directory and run model download script
cd "%DEST_DIR%\Models"

REM call download_models.bat

REM Return to the base directory
cd %BASE_DIR%

REM Set the MODELS_PATH environment variable
set "MODELS_PATH=%DEST_DIR%\Models"

REM Navigate to ros2_ws directory
cd ros2_ws || (echo "Failed to change directory to ros2_ws" & exit /b 1)

REM Source ROS 2 environment for RoboStack
call "C:\Users\marce\miniforge3\envs\ros_env\Library\share\ros2\setup.bat"

REM Build using colcon and check if it succeeded
colcon build --cmake-args "-DPython_EXECUTABLE=%CONDA_PREFIX%\python.exe -DPython3_EXECUTABLE=%CONDA_PREFIX%\python.exe -DPYTHON_EXECUTABLE=%CONDA_PREFIX%\python.exe -DPython3_FIND_STRATEGY=LOCATION -DPython_FIND_STRATEGY=LOCATION" || (echo "Colcon build failed" & exit /b 1)

REM Source the setup file after build
call install\setup.bat || (echo "Problem with sourcing install" & exit /b 1)

REM Print the PYTHONPATH
echo %PYTHONPATH%
