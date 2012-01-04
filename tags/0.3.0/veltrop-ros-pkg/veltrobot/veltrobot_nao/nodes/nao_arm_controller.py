#!/usr/bin/env python

import time
import motion
from naoqi import ALProxy
import roslib
roslib.load_manifest('veltrobot_nao')
import rospy
from geometry_msgs.msg import Point
#import naoutil

class NaoArmController():
  def LeftArmDestinationCB(self, data):
    self.ArmDestinationCB(data, "LArm")

  def RightArmDestinationCB(self, data):
    self.ArmDestinationCB(data, "RArm")

  def ArmDestinationCB(self, data, effector):
    if self.badMutex: 
      return
    self.badMutex = True

    #space      = motion.SPACE_NAO
    space      = motion.SPACE_TORSO
    axisMask   = 7          # just control position
    isAbsolute = True

    targetPos  = [data.x, data.y, data.z, 0.0, 0.0, 0.0]  
    #targetTime = 0.10
    #targetTime = 0.15
    #targetTime = 0.2
    targetTime = 0.4

    path  = [ targetPos ]
    times = [ targetTime ]

    #self.motionProxy.wbSetEffectorControl("LArm", [data.x, data.y, data.z + 0.27])
    self.motionProxy.positionInterpolation(effector, space, path,
                                           axisMask, times, isAbsolute)
    time.sleep(0.2)
    #time.sleep(0.1)
    #time.sleep(0.07)
    self.badMutex = False

  def __init__(self): 
    self.badMutex = False
    rospy.sleep(1)

    rospy.init_node('nao_arm_controller')
    
    self.ip = rospy.get_param('naoqi_ip', '127.0.0.1');
    self.port = int(rospy.get_param('naoqi_port', '9559'));

    try:
      self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
    except RuntimeError, e:
      rospy.logerr("Could not create ALMotion ALProxy: %s", e)
      exit(1)  

    rospy.Subscriber("right_arm_destination", Point, self.RightArmDestinationCB, queue_size=1)
    rospy.Subscriber("left_arm_destination", Point, self.LeftArmDestinationCB, queue_size=1)

    
    #self.motionProxy.wbEnableEffectorControl("LArm", True)
    
    self.motionProxy.setWalkArmsEnable(False, False)
    
    #self.motionProxy.setFallManagerEnabled(False)
    #naoutil.StiffnessOn(self.motionProxy)
    #naoutil.PoseInit(self.motionProxy)

if __name__ == '__main__':
  arm_controller = NaoArmController()
  rospy.spin()
  exit(0)

