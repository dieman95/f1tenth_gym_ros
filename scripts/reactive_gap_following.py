#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        
        self.scan_sub=rospy.Subscriber('scan',LaserScan,self.laser_callback,queue_size=100)


    def laser_callback(self,msg):
        ranges = msg.ranges

