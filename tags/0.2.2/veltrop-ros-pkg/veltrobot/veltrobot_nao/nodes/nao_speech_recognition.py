#!/usr/bin/env python

import motion
from naoqi import ALProxy
import roslib
roslib.load_manifest('veltrobot_nao')
import rospy
from std_msgs.msg import String
import naoutil

class NaoSpeechRecognition():
  def __init__(self): 
    rospy.sleep(1) 
    rospy.init_node('nao_speech_recognition')
    
    self.ip = rospy.get_param('naoqi_ip', '127.0.0.1');
    self.port = int(rospy.get_param('naoqi_port', '9559'));

    try:
      self.speechRecogProxy = ALProxy("ALSpeechRecognition", self.ip, self.port)
    except RuntimeError, e:
      rospy.logerr("Could not create ALSpeechRecognition ALProxy: %s", e)
      rospy.signal_shutdown("ALProxy failure")

    try:
      self.memoryProxy = ALProxy("ALMemory", self.ip, self.port)
    except RuntimeError, e:
      rospy.logerr("Could not create ALMemory ALProxy: %s", e)
      rospy.signal_shutdown("ALProxy failure")

  def __del__(self):
    self.speechRecogProxy.unsubscribe("ros")

  def run(self):
    self.speechRecogProxy.setLanguage("English")
    wordList = ["come", "go", "stand", "sit", "relax", "stiffen"]
    self.speechRecogProxy.setWordListAsVocabulary(wordList)
    self.speechRecogProxy.subscribe("ros")

    self.dataNamesList = ["WordRecognized"]
    
    self.recognizedWordPub = rospy.Publisher("recognized_word", String)   
    self.textToSpeechPub   = rospy.Publisher("speech", String)

    r = rospy.Rate(5)
    prev_word = str()
    while not rospy.is_shutdown():
      try:
        memoryData = self.memoryProxy.getListData(self.dataNamesList)
      except RuntimeError,e:
        rospy.logerr("Error accessing ALMemory: %s", e)
        rospy.signal_shutdown("No NaoQI available anymore")

      if (memoryData[0][1] >= 0.2) and (memoryData[0][0] != prev_word):
        msg = String()
        msg.data = memoryData[0][0]
        self.recognizedWordPub.publish(msg)
        msg = String()
        msg.data = "Oh kay."
        self.textToSpeechPub.publish(msg)
        prev_word = memoryData[0][0]

      # reset  
      if memoryData[0][0] == '':
        prev_word = ''

      r.sleep()

if __name__ == '__main__':
  speech_recognition = NaoSpeechRecognition()
  speech_recognition.run()
  exit(0)

