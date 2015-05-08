**************************************************** veltrop-ros-pkg ***********************************************************
Taylor Veltrop's ROS Packages.
Was actively developed between early 2010 and late 2011.
Not much out-of-the-box here for you, but plenty of stuff 

###########################
# roboard package         #
###########################
RoBoard drivers and support package for ROS.
Good out-of-box.  Sensor abstraction easy to extend if your hardware differs.
* PWM and ICS serial servo control based on joint state messages
* I2C and AD Sensor abstraction

###########################
# veltrobot package       #
###########################
For controlling the "Veltrobot" which is based on the Kondo KHR1-HV, RoBoard and various other bit & pieces.
Use this as a base for your own home built robot or NAO, but lots of modification will be needed.
* URDF
* Movement system
  * Real-time gyro based balancing
  * Pose based motion scripting
  * Bind twists and other control messages to motion scripts
  * Servo trim calibration
* GUI
  * Adjust servo trim calibration
  * Capturing robot pose for later playback
* Incomplete work for navigation and obstacle avoidance
* NAOqi API bindings, mirroring Veltrobot movement system
* Teleoperation based on Kinect, Wii remotes, PS3 controller

###########################
# virtual_reality package #
###########################
Stuff to enhance teleoperation.
If anyone out there still has a VR920 that is useful out-of-box.
The treadmill solution is based on hardware hacking, but the principal of how it is integrated could be useful.
* Vuzix VR920 support drivers and ROS abstraction node
* Control a treadmill based on Kinect data, embedded microcontroller code plus ROS node

###########################
# veltrop_stereo package  #
###########################
Stereo vision hacks.  Only useful for masochists.
* Ability to use a stereo camera if it's CCD mounting is rotated 90 degrees
* Special messages & node to send "synced" yuyv frames as a single message 
* Stuff needed for my esoteric not-fully-compatible UVC devices
