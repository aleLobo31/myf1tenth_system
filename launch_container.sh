docker run -it --privileged --net=host \
 -v /var/run/dbus/:/var/run/dbus \
 -v /dev:/dev \
 -v /etc/udev/rules.d:/etc/udev/rules.d \
 --device /dev/sensors/lidar:/dev/sensors/lidar \
 --device /dev/sensors/vesc:/dev/sensors/vesc \
 --device /dev/sensors/imu:/dev/sensors/imu \
 --device /dev/input/js0:/dev/input/js0 \
 --volume=./src/f1tenth_stack:/root/f1tenth_ws/src/f1tenth_stack  \
 --volume=./src/manual_control_pkg:/root/f1tenth_ws/src/manual_control_pkg \
 --volume=./src/safety_pkg:/root/f1tenth_ws/src/safety_pkg \
 --volume=./src/reactive_follower_pkg:/root/f1tenth_ws/src/reactive_follower_pkg \
 f1tenth-system
