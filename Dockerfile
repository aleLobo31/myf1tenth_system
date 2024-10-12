# Use the official ROS 2 Foxy base image
FROM osrf/ros2:devel

# Set up environment
#ENV LANG=C.UTF-8
#ENV LC_ALL=C.UTF-8

# Create a workspace
WORKDIR /f1tenth_ws
RUN colcon build

# Intall VESC Driver
RUN mkdir src && \ 
    cd src && \
    git clone https://github.com/ros-drivers/transport_drivers.git && \
    cd /f1tenth_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build
    
# Source the setup script. First we need to change from sh to bash to be able to source.
SHELL ["/bin/bash", "-c"] 
RUN echo "source install/setup.bash" >> ~/.bashrc
#RUN echo "source /f1tenth_ws/install/setup.bash" >> ~/.bashrc 

# Set the default command to launch a shell
CMD ["bash"]