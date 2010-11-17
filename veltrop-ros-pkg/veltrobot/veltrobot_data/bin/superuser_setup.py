#!/usr/bin/python

import os

os.system("sudo chown root:root ~/veltrop-ros-pkg/roboard/roboard_servos/bin/joint_state_controlled > /dev/null 2>&1")
os.system("sudo chmod +s ~/veltrop-ros-pkg/roboard/roboard_servos/bin/joint_state_controlled > /dev/null 2>&1")
os.system("sudo chown root:root ~/veltrop-ros-pkg/roboard/roboard_sensors/bin/poll_ad > /dev/null 2>&1")
os.system("sudo chmod +s ~/veltrop-ros-pkg/roboard/roboard_sensors/bin/poll_ad > /dev/null 2>&1")
os.system("sudo chown root:root ~/veltrop-ros-pkg/roboard/roboard_sensors/bin/poll_i2c > /dev/null 2>&1")
os.system("sudo chmod +s ~/veltrop-ros-pkg/roboard/roboard_sensors/bin/poll_i2c > /dev/null 2>&1")
os.system("sudo chown root:root ~/veltrop-ros-pkg/roboard/roboard_sensors/bin/i2c_device_manager_node > /dev/null 2>&1")
os.system("sudo chmod +s ~/veltrop-ros-pkg/roboard/roboard_sensors/bin/i2c_device_manager_node > /dev/null 2>&1")

# TODO: alas, this doesnt work...
#os.system("sudo chown root:root ~/ros/pkgs/joystick_drivers/ps3joy/ps3joy.py > /dev/null 2>&1")
#os.system("sudo chmod +s ~/ros/pkgs/joystick_drivers/ps3joy/ps3joy.py > /dev/null 2>&1")

