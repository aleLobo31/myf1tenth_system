# Use the ROS2 development base image (osrf/ros2:devel as requested)
FROM ros:foxy-ros-base-focal

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=foxy

# Update the package list and install necessary packages for building ROS2
RUN apt-get update && apt-get install -y \
    cmake \
    wget \
    curl \
    nano \
    sudo \
    libbullet-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS1
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt install ros-noetic-ros-base && \
    source /opt/ros/noetic/setup.bash && \
    apt install python3-rosinstall python3-rosinstall-generator python3-wstool

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
RUN rosdep update --include-eol-distros && rosdep install --from-paths src -i -y

# Include source command in bashrc file
RUN echo "source /opt/ros/$ROS2_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Set the entrypoint to run the container in a bash shell
CMD ["/bin/bash"]