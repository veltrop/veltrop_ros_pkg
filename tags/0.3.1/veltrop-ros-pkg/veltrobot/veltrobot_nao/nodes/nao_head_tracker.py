#!/usr/bin/env python

import math 
import motion
from naoqi import ALProxy
import roslib
roslib.load_manifest('veltrobot_nao')
import rospy
from sensor_msgs.msg import JointState
from veltrobot_msgs.msg import EnableJointGroup
#import naoutil
import thread

class NaoHeadTracker():
  def __init__(self): 
    rospy.sleep(3)

    rospy.init_node('nao_head_tracker')
    
    self.ip = rospy.get_param('naoqi_ip', '127.0.0.1');
    self.port = int(rospy.get_param('naoqi_port', '9559'));

    try:
      self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
    except RuntimeError, e:
      rospy.logerr("Could not create ALMotion ALProxy: %s", e)
      exit(1)  

    self.jointStatePub = rospy.Publisher("joint_states", JointState)
    rospy.Subscriber("enable_joint_group", EnableJointGroup, self.enableJointGroupCB, queue_size=10)
    
    self.leftArmEnabled = False
    self.rightArmEnabled = False

  def enableJointGroupCB(self, data):
    if "arms" in data.jointGroups:
      self.leftArmEnabled = self.rightArmEnabled = data.enabledStates[data.jointGroups.index("arms")]
    
    if "left_arm" in data.jointGroups:
      self.leftArmEnabled = data.enabledStates[data.jointGroups.index("left_arm")]

    if "right_arm" in data.jointGroups:
      self.rightArmEnabled = data.enabledStates[data.jointGroups.index("right_arm")]

  def run(self):
    thread.start_new_thread(rospy.spin, ())

    r = rospy.Rate(8)
    while not rospy.is_shutdown():
      headPos = self.motionProxy.getPosition("Head", motion.SPACE_TORSO, True)
      camPos = self.motionProxy.getPosition("CameraTop", motion.SPACE_TORSO, False)
      calcPos = [headPos[0], headPos[1], camPos[2]]
      leftHandPos = self.motionProxy.getPosition("LArm", motion.SPACE_TORSO, False)
      rightHandPos = self.motionProxy.getPosition("RArm", motion.SPACE_TORSO, False)
      center = [0.0, 0.0, 0,0]
      center[0] = (leftHandPos[0] + rightHandPos[0]) / 2.0
      center[1] = (leftHandPos[1] + rightHandPos[1]) / 2.0
      center[2] = (leftHandPos[2] + rightHandPos[2]) / 2.0

      if self.leftArmEnabled and self.rightArmEnabled: 
        target = center 
      elif self.leftArmEnabled:
        target = leftHandPos
      elif self.rightArmEnabled:
        target = rightHandPos
      else:
        continue

      x = target[0] - calcPos[0] # front +  back  -
      y = target[1] - calcPos[1] # left  +  right -
      z = target[2] - calcPos[2] # up    +  down  -

      #pitch = -(math.atan2(z, x) - 0.52)
      pitch = -math.atan2(z, x)
      yaw   = -math.atan2(y, x)

      #print xx, " ", yy, " ", zz, " ", pitch, "", yaw

      msg = JointState()
      msg.name.append("neck_pitch")
      msg.position.append(pitch)
      msg.velocity.append(1)
      msg.name.append("neck_yaw")
      msg.position.append(yaw)
      msg.velocity.append(1)
   
      self.jointStatePub.publish(msg)

      #space      = motion.SPACE_TORSO
      #axisMask   = motion.AXIS_MASK_WY + motion.AXIS_MASK_WZ
      #isAbsolute = True
      # negative pitch(y) looks up
      # negative yaw(z) looks right
      #targetPos  = [0, 0, 0, 0, pitch, yaw]  
      #targetTime = 0.25
      #path  = [ targetPos ]
      #times = [ targetTime ]
      #self.motionProxy.positionInterpolation("Head", space, path,
      #                                     axisMask, times, isAbsolute)

      r.sleep()

if __name__ == '__main__':
  head_tracker = NaoHeadTracker()
  head_tracker.run()
  exit(0)

