#!/bin/bash

sudo ip route add 224.0.0.0/4 dev eth0
killall -9 vrtrack
#rosrun vr920 vrtrack --multicast --invert-pitch --invert-roll --invert-yaw
rosrun vr920 vrtrack --multicast --invert-yaw
roslaunch vr920_tracker vr920_tracker.launch
killall -9 vrtrack

