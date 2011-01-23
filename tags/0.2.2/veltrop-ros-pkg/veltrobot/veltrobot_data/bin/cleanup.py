#!/usr/bin/python

import os

os.system("cd veltrobot/veltrobot_data && make clean > /dev/null 2>&1")
os.system("cd veltrobot/veltrobot_movement && make clean > /dev/null 2>&1")
os.system("cd veltrobot/veltrobot_msgs && make clean > /dev/null 2>&1")
os.system("cd veltrobot/veltrobot_sensors && make clean > /dev/null 2>&1")
os.system("cd veltrobot/veltrobot_teleop && make clean > /dev/null 2>&1")
os.system("cd veltrobot/veltrobot_gui && make clean > /dev/null 2>&1")
os.system("cd roboard/roboard_roboio && make wipe > /dev/null 2>&1")
os.system("cd roboard/roboard_servos && make clean > /dev/null 2>&1")
os.system("cd roboard/roboard_sensors && make clean > /dev/null 2>&1")
os.system("cd roboard/roboard_gui && make clean > /dev/null 2>&1")
os.system("rm `find . -name ._*` > /dev/null 2>&1")
