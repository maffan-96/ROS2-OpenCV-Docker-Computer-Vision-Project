#!/bin/bash
echo "Current working directory: $(pwd)"
# Check if the camera device exists
if [ -e /dev/video0 ]; then
    echo "Camera is connected. Launching executables/launch files."

    # Extracting arguments passed to the script
    ROS_PACKAGE=$1
    LAUNCH_FILE=$2

    # Add your command to launch executables/launch files here
    source install/setup.bash
    ros2 launch $ROS_PACKAGE $LAUNCH_FILE

else
    echo "Warning: Camera is not connected."
fi
