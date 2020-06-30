#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
import os
import rospkg


class MessageSynchronizer:
    def __init__(self):
        self.scan_debug=Subscriber('scan_debug',LaserScan)
        self.opp_scan_debug=Subscriber('opp_scan_debug',LaserScan)
        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.scan_debug,self.opp_scan_debug], queue_size = 20, slop = 0.05)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

    #callback for the synchronized messages
    def master_callback(self,scan_debug,opp_scan_debug): 
        print(scan_debug.header.frame_id,max(scan_debug.ranges))
        print(opp_scan_debug.header.frame_id,max(opp_scan_debug.ranges))
        print("--------------------------------------------------")

   

if __name__=='__main__':
    rospy.init_node('debug_sync')
    # initialize the message filter
    mf=MessageSynchronizer()
    # spin so that we can receive messages
    rospy.spin()    