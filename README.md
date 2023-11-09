# ROS2-OpenCV-Docker Computer Vision Project
 This project integrates ROS2, OpenCV, Bash scripting, and Docker for a comprehensive computer vision solution

## Project Description
A Dockerized environment is created that encapsulates the following functionalities:
1. OpenCV: A Python script for video capture, resizing, grayscale conversion, and real-time display with fps information.
2. ROS2: Foxy: A ROS 2 package with three nodes to subscribe, process, and publish images. Node 1 captures the image. Node 2 resizes images to 300x300, publishing on an output topic. Node 3 subscribes to Node 2's output, processes images using OpenCV, and publishes the results on a third ROS 2 topic.
3. Bash Script: A Bash script to check the camera connection. If the camera is connected, the script launches the ROS and OpenCV functionalities within the Docker container (or even without it). Otherwise, it provides a warning message.
4. DockerFile: A docker is used to ensure a reproducible deployment environment, encapsulating the entire computer vision pipeline. This approach promotes ease of collaboration, scalability, and consistency across different environments. Additionally, a docker-compose is additionally used to provide use of usage 

## How to use it
Run ```git clone https://github.com/maffan-96/ROS2-OpenCV-Docker-Computer-Vision-Project.git``` (assuming _git_ is already installed on your system). If not directly download the repo or install _git_ first. Now run as per your preferences from the following options specified.
   
 ### Run directly with Python (without ROS, Docker, and Bash Script)
 Navigate to the folder _ROS2-OpenCV-Docker-Computer-Vision-Project_ and run ```python3 openCV_video_capture.py```. Here, the following details are assumed
 1. Python3 and OpenCV are already installed.
 2. The system has a camera connected at ```/dev/video0``` (verify it via running a command ```v4l2-ctl --list-devices```)

The options below assume the same two details mentioned above, with a few more shown in the given cases.  
 
 ### Run via Bash script or/and ROS (without Docker)
 1. Source ROS2: Foxy via ```source /opt/ros/foxy/setup.bash``` (assuming ros2:foxy is installed). If not please install it first.
 1. Navigate to the folder _ROS2-OpenCV-Docker-Computer-Vision-Project_
 2. Run ```colcon build``` (assuming all the necessary ros2:foxy packages and OpenCV are installed).
 3. Source local ros packages ```source install/setup.bash```
 4. Run ```bash check_camera.py``` or directly run via ROS2 via command ```ros2 launch video_capture image_processing.launch.py``` (_video_capture_ is the name of package)
 
  ### Run via Docker
  1. Ensure that the ```docker``` and ```docker-compose``` are installed. If not, install it first. For reference, the Docker version used is 24.0.6.
  2. Navigate to the folder _ROS2-OpenCV-Docker-Computer-Vision-Project_
  3. Run command ```docker-compose build```
  4. After a successful build, go to another terminal and type ```xhost +local:root```
  5. Run command ```docker-compose up```
  6. It will open a rviz2 visualizer where you can choose to view the topic of your choice. These three topics are:
      1.  ```input_image_topic``` (frames from the camera are published as it is on this topic, i.e., original video)
      2.  ```output_image_topic``` (frames from the _input_image_topic_ is published after resizing on this topic, i.e., resized video)
      3.  ```processed_image_topic``` (frames from the _output_image_topic_ is published after grayscale conversion on this topic, i.e., processed video)
  7. In order to close, run command ```docker-compose down```
    
 
    

    

    
         
     
