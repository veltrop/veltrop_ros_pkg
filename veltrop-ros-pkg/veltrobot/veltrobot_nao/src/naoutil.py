# this is from config.py of the nao sdk

import naoqi
import motion
from naoqi import ALProxy

IP = "127.0.0.1" # set your Ip adress here

PORT = 9559

if (IP == ""):
  print "IP address not defined, aborting"
  print "Please define it in " + __file__
  exit(1)

def loadProxy(pName):
  print "---------------------"
  print "Loading proxy"
  print "---------------------"
  proxy = ALProxy(pName, IP, PORT)
  print "---------------------"
  print "Starting " + pName + " Tests"
  print "---------------------"
  return proxy

def StiffnessOn(proxy):
  #We use the "Body" name to signify the collection of all joints
  pNames = "Body"
  pStiffnessLists = 1.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def PoseInit(proxy):
  # Define The Initial Position
  HeadYawAngle       = 0
  HeadPitchAngle     = 0

  ShoulderPitchAngle = 80  
  ShoulderRollAngle  = 20  
  ElbowYawAngle      = -80 
  ElbowRollAngle     = -60 
  WristYawAngle      = 0
  HandAngle          = 0
  
  HipYawPitchAngle   = 0
  HipRollAngle       = 0
  HipPitchAngle      = -25 
  KneePitchAngle     = 40  
  AnklePitchAngle    = -20 
  AnkleRollAngle     = 0
  
  # Get the Robot Configuration
  robotConfig = proxy.getRobotConfig()
  
  if (robotConfig[1][0] == "naoAcademics"):
  
      Head     = [HeadYawAngle, HeadPitchAngle]
      
      LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle, WristYawAngle, HandAngle]
      RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle, WristYawAngle, HandAngle]
      
      LeftLeg  = [HipYawPitchAngle, +HipRollAngle, HipPitchAngle, KneePitchAngle, AnklePitchAngle, +AnkleRollAngle]
      RightLeg = [HipYawPitchAngle, -HipRollAngle, HipPitchAngle, KneePitchAngle, AnklePitchAngle, -AnkleRollAngle]

  elif (robotConfig[1][0] == "naoRobocup"):

      Head     = [HeadYawAngle, HeadPitchAngle]
      
      LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle]
      RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle]
      
      LeftLeg  = [HipYawPitchAngle, +HipRollAngle, HipPitchAngle, KneePitchAngle, AnklePitchAngle, +AnkleRollAngle]
      RightLeg = [HipYawPitchAngle, -HipRollAngle, HipPitchAngle, KneePitchAngle, AnklePitchAngle, -AnkleRollAngle]
  
  elif (robotConfig[1][0] == "naoT14"):

      Head     = [HeadYawAngle, HeadPitchAngle]
      
      LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle, WristYawAngle, HandAngle]
      RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle, WristYawAngle, HandAngle]
      
      LeftLeg  = []
      RightLeg = []

  elif (robotConfig[1][0] == "naoT2"):

      Head     = [HeadYawAngle, HeadPitchAngle]
      
      LeftArm  = []
      RightArm = []
      
      LeftLeg  = []
      RightLeg = []

  # Gather the joints together
  pTargetAngles = Head + LeftArm + LeftLeg + RightLeg + RightArm

  # Convert to radians
  pTargetAngles = [ x * motion.TO_RAD for x in pTargetAngles]

  #------------------------------ send the commands -----------------------------
  # We use the "Body" name to signify the collection of all joints
  pNames = "Body"
  # We set the fraction of max speed
  pMaxSpeedFraction = 0.2
  # Ask motion to do this with a blocking call
  proxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)
