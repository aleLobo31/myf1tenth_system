# Creamos una norma para el LIDAR
touch /etc/udev/rules.d/99-lidar.rules
echo "SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="sensors/lidar"" | sudo tee /etc/udev/rules.d/99-lidar.rules > /dev/null

# Creamos una norma para el VESC
touch /etc/udev/rules.d/99-vesc.rules
echo "SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"" | sudo tee /etc/udev/rules.d/99-vesc.rules > /dev/null

# Creamos una norma para la IMU
touch /etc/udev/rules.d/99-imu.rules
echo "SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout", SYMLINK+="sensors/imu"" | sudo tee /etc/udev/rules.d/99-imu.rules > /dev/null

# Cargamos las nuevas reglas y las aplicamos
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER

