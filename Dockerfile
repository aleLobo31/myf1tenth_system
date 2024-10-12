# Use the official ROS 2 Foxy base image
FROM ros:foxy-ros-base

# Set up environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install necessary tools: colcon, rosdep, and vcs
RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    && rm -rf /var/lib/apt/lists/*

# Initialize and update rosdep
RUN rosdep update

# Create a workspace
WORKDIR /f1tenth_ws
RUN colcon build

# Intall VESC Driver
RUN cd src/ && \
    git clone https://github.com/ros-drivers/transport_drivers.git && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build && \
    git clone https://github.com/f1tenth/vesc.git && \
    
# Source the setup script. First we need to change from sh to bash to be able to source.
SHELL ["/bin/bash", "-c"] 
RUN echo "source install/setup.bash" >> ~/.bashrc
#RUN echo "source /f1tenth_ws/install/setup.bash" >> ~/.bashrc 

# Set the default command to launch a shell
CMD ["bash"]