# Creamos una norma para el LIDAR
touch /etc/udev/rules.d/99-lidar.rules
echo "SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="sensors/lidar"" | sudo tee /etc/udev/rules.d/99-lidar.rules > /dev/null

# Creamos una norma para el VESC
touch /etc/udev/rules.d/99-vesc.rules
echo "SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"" | sudo tee /etc/udev/rules.d/99-vesc.rules > /dev/null

# Creamos una norma para el JOYSTICK
touch /etc/udev/rules.d/99-joypad.rules
echo "SUBSYSTEM=="hidraw", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c52b", MODE="0666", GROUP="dialout", SYMLINK+="input/joypad-f710"" | sudo tee /etc/udev/rules.d/99-joypad.rules > /dev/null

# Cargamos las nuevas reglas y las aplicamos
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER

