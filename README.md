# ROS2, OpenCV, & Docker Computer Vision Project

This project integrates ROS2, OpenCV, Bash scripting, and Docker for a comprehensive computer vision solution.

## Project Description

A Dockerized environment is created that encapsulates the following functionalities:

1. **OpenCV:**
   - A Python script for video capture, resizing, grayscale conversion, and real-time display with fps information.

2. **ROS2: Foxy:**
   - A ROS 2 package with three nodes to subscribe, process, and publish images.
     - Node 1 captures the image.
     - Node 2 resizes images to 300x300, publishing on an output topic.
     - Node 3 subscribes to Node 2's output, processes images using OpenCV, and publishes the results on a third ROS 2 topic.

3. **Bash Script:**
   - A Bash script to check the camera connection. If the camera is connected, the script launches the ROS and OpenCV functionalities within the Docker container (or even without it). Otherwise, it provides a warning message.

4. **DockerFile:**
   - A Docker is used to ensure a reproducible deployment environment, encapsulating the entire computer vision pipeline. This approach promotes ease of collaboration, scalability, and consistency across different environments. Additionally, a docker-compose is used to provide usage.

## How to Use

1. **Run Directly with Python (without ROS, Docker, and Bash Script):**
   - Navigate to the folder _ROS2-OpenCV-Docker-Computer-Vision-Project_.
   - Run `python3 openCV_video_capture.py`.
   - Assumes Python3 and OpenCV are already installed, and a camera is connected at `/dev/video0` (verify it via running the command `v4l2-ctl --list-devices`).

2. **Run via Bash Script or/and ROS (without Docker):**
   - Source ROS2: Foxy via `source /opt/ros/foxy/setup.bash` (assuming ros2:foxy is installed).
   - Navigate to the folder _ROS2-OpenCV-Docker-Computer-Vision-Project_.
   - Enter command `colcon build` (assuming all the necessary ros2:foxy packages and OpenCV are installed).
   - Source local ros packages `source install/setup.bash`.
   - Enter command `bash check_camera.sh video_capture image_processing.launch.py` or directly run via ROS2 using the command `ros2 launch video_capture image_processing.launch.py` (_video_capture_ is the name of the package and _image_processing.launch.py_ is a name of the launch file).

3. **Run via Docker:**
   - Ensure that `docker` and `docker-compose` are installed. If not, install them first (Docker version used: 24.0.6).
   - Navigate to the folder _ROS2-OpenCV-Docker-Computer-Vision-Project_.
   - Run `docker-compose build`.
   - After a successful build, go to another terminal and type `xhost +local:root`.
   - Run `docker-compose up`.
   - An rviz2 visualizer will open, and the following topics could be viewed by selection:
      - `input_image_topic` (original video frames),
      - `output_image_topic` (resized video frames),
      - `processed_image_topic` (grayscale processed video frames).
   - To close, run `docker-compose down`.

