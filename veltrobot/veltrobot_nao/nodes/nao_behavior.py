#!/usr/bin/env python

from naoqi import ALProxy
import roslib
roslib.load_manifest('veltrobot_nao')
import rospy
from std_msgs.msg import String

class NaoBehavior():
  def __init__(self): 
    #rospy.sleep(2) 
    rospy.init_node('nao_behavior')
    
    self.ip = rospy.get_param('naoqi_ip', '127.0.0.1');
    self.port = int(rospy.get_param('naoqi_port', '9559'));

    try:
      self.behaviorProxy = ALProxy("ALBehaviorManager", self.ip, self.port)
    except RuntimeError, e:
      rospy.logerr("Could not create ALBehaviorManager ALProxy: %s", e)
      rospy.signal_shutdown("ALProxy failure")
    
    rospy.Subscriber("recognized_word", String, self.RecognizedWordCB, queue_size=1)
    rospy.Subscriber("motion_name", String, self.MotionNameCB, queue_size=1)
 
  def RecognizedWordCB(self, data):
    #print data.data
    if self.behaviorProxy.isBehaviorInstalled(data.data):
      self.behaviorProxy.runBehavior(data.data)

  def MotionNameCB(self, data):
    #print self.behaviorProxy.getInstalledBehaviors()
    #print self.behaviorProxy.getRunningBehaviors()
    if self.behaviorProxy.isBehaviorInstalled(data.data):
      #self.behaviorProxy.stopAllBehaviors()
      self.behaviorProxy.runBehavior(data.data)

if __name__ == '__main__':
  behavior = NaoBehavior()
  rospy.spin()
  exit(0)

