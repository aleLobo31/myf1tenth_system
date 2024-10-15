# Use the ROS2 development base image (osrf/ros2:devel as requested)
# FROM osrf/ros2:devel
FROM ros:foxy-ros-base-focal

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS2_DISTRO=foxy
ENV ROS1_DISTRO=noetic

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

RUN git clone https://github.com/ros-drivers/transport_drivers.git && \
    git clone https://github.com/f1tenth/vesc.git

WORKDIR /root/f1tenth_ws

RUN wget https://github.com/chriskohlhoff/asio/archive/asio-1-12-2.tar.gz && \
    tar -xvzf asio-1-12-2.tar.gz && \
    cd asio-asio-1-12-2 && \
    cp -r asio/include/asio /usr/include/ && \
    apt-get update
   

RUN rosdep update --include-eol-distros && rosdep install --from-paths src -i -y --skip-keys="serial message_generation tf catkin roscpp message_runtime nodelet roslint"

#RUN sudo apt-get install libserial-dev

# Install ROS 1 Noetic (for ROS 1 dependencies)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-catkin \
    ros-noetic-serial \
    ros-noetic-message-generation \
    ros-noetic-tf \
    ros-noetic-roscpp \
    ros-noetic-message-runtime \
    ros-noetic-nodelet \
    ros-noetic-roslint

# Source ROS 1 and Ros 2 setup in bashrc
RUN echo "source /opt/ros/$ROS1_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/$ROS2_DISTRO/setup.bash" >> ~/.bashrc

# Install ros1_bridge
RUN apt-get update && apt-get install -y \
    ros-foxy-ros1-bridge \
    && rm -rf /var/lib/apt/lists/*

# Set the entrypoint to run the container in a bash shell
CMD ["/bin/bash"]
