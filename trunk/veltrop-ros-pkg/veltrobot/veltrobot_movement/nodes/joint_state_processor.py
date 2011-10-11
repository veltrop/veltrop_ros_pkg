#!/usr/bin/env python

import roslib
roslib.load_manifest('veltrobot_movement')
import rospy
from sensor_msgs.msg import JointState

class JointStateProcessor():
  def __init__(self): 
    rospy.init_node('joint_state_processor')

    rospy.Subscriber("joint_states_in", JointState, self.JointStateCB, queue_size=1)
    self.JointStatePub = rospy.Publisher("joint_states_out", JointState)   

    self.conversions = rospy.get_param('~conversions');
    
  def JointStateCB(self, data):
    msg = JointState()
    
    for conversion in self.conversions:
      if conversion['input_joint'] in data.name:
        msg.name.append(conversion['output_joint'])
        position = data.position[data.name.index(conversion['input_joint'])]
        position = position * conversion['multiply'] + conversion['add']
        msg.position.append(position)
    
    if len(msg.name) is not 0:
      self.JointStatePub.publish(msg)

if __name__ == '__main__':
  joint_state_processor = JointStateProcessor()
  rospy.spin()
  exit(0)

