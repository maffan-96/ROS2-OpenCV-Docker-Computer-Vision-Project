# Use the official ROS 2 Foxy image as the base image

FROM osrf/ros:foxy-desktop

# Install necessary tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    && apt-get -y install python3-pip \
	&& pip install --upgrade setuptools \
    && rm -rf /var/lib/apt/lists/

RUN apt-get update \
	&& apt upgrade -y \
    && apt-get install -y x11-apps \
    && sudo apt-get install -y --reinstall libxcb-xinerama0 \ 
    # new line above "reinstall"
    && apt-get install -y xauth \
	&& apt-get install -q -y \
		ros-foxy-cv-bridge \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/*


# Setup the workspace
WORKDIR /ros2_ws

# Create a ROS 2 workspace and initialize it
RUN mkdir src
COPY video_capture /ros2_ws/src/video_capture/

# Check if the default sources list file exists and delete it if necessary
RUN sources_list=/etc/ros/rosdep/sources.list.d/20-default.list \
    && if [ -f "$sources_list" ]; then rm -f "$sources_list"; fi

# Initialize rosdep and update
RUN rosdep init && rosdep update

# Install ROS 2 package dependencies
RUN apt-get update && rosdep install --from-paths src --ignore-src -y -r \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y

# Build the ROS 2 package
RUN colcon build

# Install OpenCV
RUN apt-get update && apt-get install -y python3-opencv
# new qt5 above "install"

# Source the ROS 2 workspace setup file
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Copy the launch file to the container
#COPY ./video_capture/launch/image_processing.launch.py /ros2_ws/src/video_capture/launch/image_processing.launch.py
# Set the default launch file
ENV ROS_DOMAIN_ID=0
ENV LAUNCH_FILE_NAME=image_processing.launch.py

# Copy the check_camera.sh script into the container
COPY check_camera.sh .

# Set execute permissions for the script
RUN chmod +x check_camera.sh

# Start the ROS 2 package and OpenCV in the container via bash file
CMD ["bash", "./check_camera.sh", "video_capture", "image_processing.launch.py"]
# Start the ROS 2 package and OpenCV in the container
#CMD [ "bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch video_capture $LAUNCH_FILE_NAME" ]
#CMD ["/bin/sh", "-c", "bash"]
