#!/usr/bin/env python

from naoqi import ALProxy
import roslib
roslib.load_manifest('veltrobot_nao')
import rospy
from sensor_msgs.msg import JointState
#import naoutil

PI = 3.14159265359
HALFPI = 1.57079632679
QUARTPI = 0.785398163397

class NaoJointController():
  def JointStateCB(self, data):

    names = list()
    angles = list()

    for name in data.name:
      names.append(name)

    for position in data.position:
      angles.append(position)

    if len(names) is not 0:
      fractionMaxSpeed = 0.999 #TODO: something better than this to respect timing...
      self.motionProxy.setAngles(names, angles, fractionMaxSpeed)

    #if len(names) is not 0:
    #  fractionMaxSpeed = 0.9
    #  self.motionProxy.setAngles(names, angles, fractionMaxSpeed)

  def __init__(self): 
    rospy.init_node('nao_joint_controller')
    
    self.ip = rospy.get_param('naoqi_ip', '127.0.0.1');
    self.port = int(rospy.get_param('naoqi_port', '9559'));

    try:
      self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
    except RuntimeError, e:
      rospy.logerr("Could not create ALMotion ALProxy: %s", e)
      exit(1)  

    rospy.Subscriber("joint_states", JointState, self.JointStateCB, queue_size=1)
 	
    self.motionProxy.setWalkArmsEnable(False, False)
    #naoutil.StiffnessOn(self.motionProxy)
    #naoutil.PoseInit(self.motionProxy)


if __name__ == '__main__':
  joint_controller = NaoJointController()
  rospy.spin()
  exit(0)

