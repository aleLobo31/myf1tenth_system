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

RUN sudo apt-get install ros-foxy-catkin ros-foxy-tf ros-foxy-message-generation ros-foxy-serial ros-foxy-roscpp ros-foxy-message_runtime ros-foxy-nodelet ros-foxy-roslint

# Source the setup script
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Set the entrypoint to run the container in a bash shell
CMD ["/bin/bash"]
