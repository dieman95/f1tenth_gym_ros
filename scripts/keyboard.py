#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty
pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=100)

keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key


speed = 1.5
turn = 0.25


if __name__=="__main__":

  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('keyboard', anonymous=True)

  x = 0
  th = 0
  status = 0

  try:
    while(not rospy.is_shutdown()):
       key = getKey()
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break
       msg = drive = AckermannDriveStamped()
       msg.drive.steering_angle = th*turn
       msg.drive.speed = x*speed
       pub.publish(msg)

  except:
    print 'error'

  finally:
      msg = AckermannDriveStamped()
      msg.drive.steering_angle = 0
      msg.drive.speed = 0
      pub.publish(msg)
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
