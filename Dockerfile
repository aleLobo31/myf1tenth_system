# Use the ROS2 development base image (osrf/ros2:devel as requested)
FROM ros:foxy-ros-base-focal

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Update the package list and install necessary packages for building ROS2
RUN apt-get update && apt-get install -y \
    cmake \
    wget \
    curl \
    nano \
    sudo \
    libbullet-dev \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /root/f1tenth_ws/src

# Install VESC Drivers
RUN git clone https://github.com/ros-drivers/transport_drivers.git && \
    git clone -b foxy https://github.com/f1tenth/vesc.git

WORKDIR /root/f1tenth_ws

# Set source file to install asio dependency later on
RUN wget https://github.com/chriskohlhoff/asio/archive/asio-1-12-2.tar.gz && \
    tar -xvzf asio-1-12-2.tar.gz && \
    cd asio-asio-1-12-2 && \
    cp -r asio/include/asio /usr/include/ && \
    apt-get update
   
# Install the associated dependencies
RUN rosdep update --include-eol-distros && rosdep install --from-paths src -i -y 

# Change to src folder to install required ros packages
WORKDIR /root/f1tenth_ws/src/ldlidar

# Install LIDAR Drivers
RUN git clone -b ros2 https://github.com/linorobot/ldlidar.git

WORKDIR /root/f1tenth_ws

RUN rosdep update --include-eol-distros && rosdep install --from-path src --ignore-src -y

WORKDIR /root/f1tenth_ws

# Install Joy Ros package
RUN apt-get update && apt-get install -y ros-foxy-joy 

# Install other required dependencies
RUN apt install -y ros-foxy-diagnostics

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/foxy/setup.bash && colcon build

# Set the entrypoint to run the container in a bash shell
CMD ["/bin/bash"]
