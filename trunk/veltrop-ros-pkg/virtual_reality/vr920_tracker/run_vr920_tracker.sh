#!/bin/bash

killall -9 vrtrack
#rosrun vr920 vrtrack --multicast --invert-pitch --invert-roll --invert-yaw
rosrun vr920 vrtrack --multicast --invert-yaw
roslaunch vr920_tracker vr920_tracker.launch
killall -9 vrtrack

