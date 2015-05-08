#!/usr/bin/env python

import motion
from naoqi import ALProxy
import roslib
roslib.load_manifest('veltrobot_nao')
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Point
#import naoutil

# TODO: really need services here instead of simple topics.

class NaoNavigationController():
  def RecognizedWordCB(self, data):
    if data == "go":
      self.GoPointCB(Empty())

  def PointingDestinationCB(self, data):
    self.pointingDestination = data

  def TorsoDestinationCB(self, data):
    self.torsoDestination = data   

  def GoPointCB(self, data):
    print "going"    

  def GoTorsoCB(self, data):
    RobotPosition = self.motionProxy.getRobotPosition(False)
    naoPos = Point()
    naoPos.x = RobotPosition[0]
    naoPos.y = RobotPosition[1]
    naoPos.z = RobotPosition[2]
    
    go = Point()
    go.x = (naoPos.x - self.naoCalib.x) + (self.torsoDestination.x - self.torsoCalib.x)
    go.y = (naoPos.y - self.naoCalib.y) + (self.torsoDestination.y - self.torsoCalib.y)
    go.z = (naoPos.z - self.naoCalib.z) + (self.torsoDestination.z - self.torsoCalib.z)

    self.motionProxy.walkTo(go.x, go.y, go.z)

    #self.TorsoCalibrationCB(Empty())

    #X = go.x - self.prevGo.x
    #Y = go.y - self.prevGo.y
    #Theta = go.z - self.prevGo.z
    #Frequency = 1.0
    #self.motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    #self.prevGo = go

  def TorsoCalibrationCB(self, data):
    RobotPosition = self.motionProxy.getRobotPosition(False)
    self.naoCalib.x = RobotPosition[0]
    self.naoCalib.y = RobotPosition[1]
    self.naoCalib.z = RobotPosition[2]
    self.torsoCalib = self.torsoDestination
     
  def __init__(self): 
    rospy.sleep(1)

    rospy.init_node('nao_navigation_controller')
    
    self.ip = rospy.get_param('naoqi_ip', '127.0.0.1');
    self.port = int(rospy.get_param('naoqi_port', '9559'));

    try:
      self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
    except RuntimeError, e:
      rospy.logerr("Could not create ALMotion ALProxy: %s", e)
      exit(1)

    rospy.Subscriber("torso_destination", Point, self.TorsoDestinationCB, queue_size=1)
    rospy.Subscriber("pointing_destination", Point, self.PointingDestinationCB, queue_size=1)
    rospy.Subscriber("calibrate_torso", Empty, self.TorsoCalibrationCB, queue_size=1)
    rospy.Subscriber("goto_torso_destination", Empty, self.GoTorsoCB, queue_size=1)
    rospy.Subscriber("goto_pointing_destination", Empty, self.GoPointCB, queue_size=1)
    rospy.Subscriber("recognized_word", String, self.RecognizedWordCB, queue_size=1)

    self.motionProxy.setWalkArmsEnable(False, False)

    self.torsoDestination = Point()
    self.pointingDestination = Point()
    self.torsoCalib = Point()
    self.naoCalib = Point()
    #self.prevGo = Point()

if __name__ == '__main__':
  controller = NaoNavigationController()
  rospy.spin()
  exit(0)

