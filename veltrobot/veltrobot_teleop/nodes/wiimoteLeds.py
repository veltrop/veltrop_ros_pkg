#!/usr/bin/env python

import roslib; roslib.load_manifest('wiimote')
import rospy
import time
from wiimote.msg import LEDControl
from wiimote.msg import TimedSwitch
#from wiimote.msg import State

def talker():
    # Send one message, or keep repeating?
    # oneShot = False
    
    pub = rospy.Publisher('/wiimote/leds', LEDControl)
    rospy.init_node('wiimoteLeds', anonymous=True)
    
    onSwitch       = TimedSwitch(switch_mode=TimedSwitch.ON)
    offSwitch      = TimedSwitch(switch_mode=TimedSwitch.OFF)
    # noChangeSwitch = TimedSwitch(switch_mode=TimedSwitch.NO_CHANGE)
    
    timed_switch_array=[offSwitch, offSwitch, offSwitch, offSwitch]
    
    led0 = rospy.get_param('~led0', False);
    led1 = rospy.get_param('~led1', False);
    led2 = rospy.get_param('~led2', False);
    led3 = rospy.get_param('~led3', False);
    if led0:
      timed_switch_array[0] = onSwitch
    if led1:
      timed_switch_array[1] = onSwitch
    if led2:
      timed_switch_array[2] = onSwitch
    if led3:
      timed_switch_array[3] = onSwitch

    msg0 = LEDControl(timed_switch_array)

    while not rospy.is_shutdown():
      pub.publish(msg0)
      time.sleep(1)

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
