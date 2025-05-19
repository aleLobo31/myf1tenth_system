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
    bluetooth \
    bluez \
    bluez-tools \
    python3-pip \
    python3-dev \
    python3-numpy \
    cython3 \
    libhidapi-dev \
    libusb-1.0-0-dev \
    dbus \
    && rm -rf /var/lib/apt/lists/*

#Install ds4drv
RUN pip3 install ds4drv

# After installing ds4drv
COPY scripts/ds4led /usr/local/bin/
RUN chmod +x /usr/local/bin/ds4led

#Enable Bluetooth permissions for the container
RUN usermod -aG bluetooth root

WORKDIR /root/f1tenth_ws

# Set source file to install asio dependency later on
RUN wget https://github.com/chriskohlhoff/asio/archive/asio-1-12-2.tar.gz && \
    tar -xvzf asio-1-12-2.tar.gz && \
    cd asio-asio-1-12-2 && \
    cp -r asio/include/asio /usr/include/ && \
    apt-get update

# Create a workspace directory
WORKDIR /root/f1tenth_ws/src

# Install VESC Drivers
RUN git clone https://github.com/ros-drivers/transport_drivers.git && \
    git clone -b foxy https://github.com/f1tenth/vesc.git
   
# Install LIDAR Drivers
RUN git clone https://github.com/rudislabs/ldlidar_stl_ros2.git

# Install IMU
RUN git clone https://github.com/JaimeG-ELC/razor_imu_ros2.git


#Install Particle Filter repo
RUN git clone -b foxy-devel https://github.com/f1tenth/particle_filter.git

WORKDIR /root/f1tenth_ws/src/particle_filter
RUN apt-get update && apt-get install -y python3-dev build-essential && \
    pip3 install cython && \
    git clone -b foxy-devel https://github.com/f1tenth/range_libc.git

WORKDIR /root/f1tenth_ws/src/particle_filter/range_libc/pywrapper
RUN chmod +x compile.sh && ./compile.sh

WORKDIR /root/f1tenth_ws

# Install the associated dependencies
# RUN rosdep update --include-eol-distros && rosdep install --from-paths src -i -y 

RUN rosdep update --include-eol-distros && rosdep install --from-path src --ignore-src -y

# Install Joy Ros package
RUN apt-get update && apt-get install -y ros-foxy-joy 

# Install other required dependencies
RUN apt install -y ros-foxy-diagnostics

#Install SLAM toolbox
RUN apt install -y ros-foxy-slam-toolbox

#Install RVIZ2
RUN apt install -y ros-foxy-rviz2

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/foxy/setup.bash && colcon build

# Set the entrypoint to run the container in a bash shell
CMD ["/bin/bash"]
