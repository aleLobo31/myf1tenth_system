# Use the ROS2 development base image (osrf/ros2:devel as requested)
FROM ros:foxy-ros-base-focal

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
#ENV ROS1_DISTRO=noetic
#ENV ROS2_DISTRO=foxy

# Update the package list and install necessary packages for building ROS2
RUN apt-get update && apt-get install -y \
    cmake \
    wget \
    curl \
    nano \
    sudo \
    libbullet-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /root/f1tenth_ws/src

# Install VESC Drivers
RUN git clone https://github.com/ros-drivers/transport_drivers.git && \
    git clone https://github.com/f1tenth/vesc.git

WORKDIR /root/f1tenth_ws

# Set source file to install asio dependency later on
RUN wget https://github.com/chriskohlhoff/asio/archive/asio-1-12-2.tar.gz && \
    tar -xvzf asio-1-12-2.tar.gz && \
    cd asio-asio-1-12-2 && \
    cp -r asio/include/asio /usr/include/ && \
    apt-get update
   
# Install the associated dependencies
RUN rosdep update --i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nclude-eol-distros && rosdep install --from-paths src -i -y --skip-keys="serial message_generation tf catkin roscpp message_runtime nodelet roslint”

# Install ROS1
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt-get update && \ 
    apt install -y ros-noetic-ros-base && \
    apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool \
    ros-noetic-catkin \
    ros-noetic-serial \
    ros-noetic-message-generation \
    ros-noetic-tf \
    ros-noetic-roscpp \
    ros-noetic-message-runtime \
    ros-noetic-nodelet \
    ros-noetic-roslint \
    && rm -rf /var/lib/apt/lists/*

# Install ros1_bridge
RUN apt-get update && apt-get install -y \
    ros-foxy-ros1-bridge \
    && rm -rf /var/lib/apt/lists/*

# Include source command in bashrc file
#RUN echo "source /opt/ros/$ROS2_DISTRO/setup.bash" >> ~/.bashrc
#RUN echo "source /opt/ros/$ROS1_DISTRO/setup.bash" >> ~/.bashrc 

# Set the entrypoint to run the container in a bash shell
CMD ["/bin/bash"]