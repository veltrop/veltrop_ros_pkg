#!/bin/bash

rosparam load `rospack find veltrobot_data`/conf/servos.yaml /joint_state_controlled 
rostopic pub /update_config std_msgs/Empty -1
rostopic pub /motion_name std_msgs/String init -1
