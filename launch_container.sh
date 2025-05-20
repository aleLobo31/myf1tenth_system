docker run -it --privileged --net=host \
    -v /var/run/dbus/:/var/run/dbus \
    -v /dev:/dev \
    -v /etc/udev/rules.d:/etc/udev/rules.d \
    --device /dev/sensors/lidar:/dev/sensors/lidar \
    --device /dev/sensors/vesc:/dev/sensors/vesc \
    --device /dev/sensors/imu:/dev/sensors/imu \
    --device /dev/input/js0:/dev/input/js0 \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=9 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/.Xauthority:/root/.Xauthority \
    --volume=./src/f1tenth_stack:/root/f1tenth_ws/src/f1tenth_stack  \
    --volume=./src/manual_control_pkg:/root/f1tenth_ws/src/manual_control_pkg \
    --volume=./src/safety_pkg:/root/f1tenth_ws/src/safety_pkg \
    --volume=./src/reactive_follower_pkg:/root/f1tenth_ws/src/reactive_follower_pkg \
    --volume=./src/waypoint_generator:/root/f1tenth_ws/src/waypoint_generator \
    --volume=./src/pure_pursuit:/root/f1tenth_ws/src/pure_pursuit \
    f1tenth-system
