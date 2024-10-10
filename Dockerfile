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

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Create a workspace
# WORKDIR /ros2_ws
RUN mkdir -p f1tenth_ws/src && \
    cd f1tenth_ws && \
    colcon build

RUN cd src && \
    git clone https://github.com/f1tenth/f1tenth_system.git && \
    git submodule update --init --force --remote

RUN cd $HOME/f1tenth_ws && \
    rosdep update && \
    rosdep install --from-paths src -i -y && \
    colcon build

# Source the setup script. First we need to change from sh to bash to be able to source.
SHELL ["/bin/bash", "-c"] 
RUN echo "source /f1tenth_ws/install/setup.bash" >> ~/.bashrc

# Set the default command to launch a shell
CMD ["bash"]