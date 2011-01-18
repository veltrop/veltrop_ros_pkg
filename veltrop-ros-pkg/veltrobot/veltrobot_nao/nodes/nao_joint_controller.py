#!/usr/bin/env python

from naoqi import ALProxy
import roslib
roslib.load_manifest('veltrobot_nao')
import rospy
from sensor_msgs.msg import JointState

PI = 3.14159265359
HALFPI = 1.57079632679
QUARTPI = 0.785398163397

class NaoJointController():
  def JointStateCB(self, data):
    names = list()
    angles = list()

    # convert veltrobot joint names and zero-positions to nao

    if "shoulder_left_pitch" in data.name:
      names.append('LShoulderPitch')
      angles.append(-data.position[data.name.index("shoulder_left_pitch")] + HALFPI)
   
    if "shoulder_left_roll" in data.name:
      names.append('LShoulderRoll')
      angles.append(data.position[data.name.index("shoulder_left_roll")])
    
    if "shoulder_left_yaw" in data.name:
      names.append('LElbowYaw')
      angles.append(-data.position[data.name.index("shoulder_left_yaw")])
    
    if "elbow_left_roll" in data.name:
      names.append('LElbowRoll')
      angles.append(data.position[data.name.index("elbow_left_roll")])
    
    if "shoulder_right_pitch" in data.name:
      names.append('RShoulderPitch')
      angles.append(data.position[data.name.index("shoulder_right_pitch")] + HALFPI)
    
    if "shoulder_right_roll" in data.name:
      names.append('RShoulderRoll')
      angles.append(data.position[data.name.index("shoulder_right_roll")])
    
    if "shoulder_right_yaw" in data.name:
      names.append('RElbowYaw')
      angles.append(-data.position[data.name.index("shoulder_right_yaw")])
    
    if "elbow_right_roll" in data.name:
      names.append('RElbowRoll')
      angles.append(data.position[data.name.index("elbow_right_roll")]) 
   
    if "wrist_left_yaw" in data.name:
      names.append('LWristYaw')
      angles.append(data.position[data.name.index("wrist_left_yaw")])
    
    if "wrist_right_yaw" in data.name:
      names.append('RWristYaw')
      angles.append(data.position[data.name.index("wrist_right_yaw")]) 

    if "hand_right" in data.name:
      names.append('RHand')
      angles.append(data.position[data.name.index("hand_right")])
    
    if "hand_left" in data.name:
      names.append('LHand')
      angles.append(data.position[data.name.index("hand_left")])  

    if names.len() is not 0:
      fractionMaxSpeed = 0.9
      self.motionProxy.setAngles(names, angles, fractionMaxSpeed)

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
 	
if __name__ == '__main__':
  joint_controller = NaoJointController()
  rospy.spin()
  exit(0)

