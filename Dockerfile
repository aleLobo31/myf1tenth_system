# Use the ROS2 development base image (osrf/ros2:devel as requested)
# FROM osrf/ros2:devel
FROM ros:foxy-ros-base-focal

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=foxy

# Update the package list and install necessary packages for building ROS2
RUN apt-get update && apt-get install -y \
    cmake \
    wget \
    curl \
    nano \
    sudo \
    libbullet-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS2 apt repository
# RUN curl -sSL http://repo.ros2.org/repos.key | sudo apt-key add - && \
#    echo "deb http://packages.ros2.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list

# Create a workspace directory
WORKDIR /root/f1tenth_ws/src

RUN git clone https://github.com/ros-drivers/transport_drivers.git && \
    git clone https://github.com/f1tenth/vesc.git

WORKDIR /root/f1tenth_ws

RUN rosdep update --include-eol-distros && rosdep install --from-paths src -i -y

# Source the setup script
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /root/f1tenth_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to run the container in a bash shell
ENTRYPOINT ["/bin/bash"]