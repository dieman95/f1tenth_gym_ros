#!/usr/bin/env python
import rospy 
from f1tenth_gym_ros.msg import StampedBool
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np

class Decision_Manager:
    def __init__(self,safety_topic,drive_topic):

        # Initialize subscribers
        self.scan_subscriber=Subscriber(safety_topic,StampedBool,queue_size=100)
        self.odom_subscriber=Subscriber(drive_topic,AckermannDriveStamped,queue_size=100)

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.scan_subscriber,self.odom_subscriber], queue_size = 100, slop = 0.020)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

    
    def master_callback(self,bool_topic,drive_topic):
        print(bool_topic)
        print(drive_topic)




if __name__ == "__main__":
    #get the arguments passed from the launch file
    
    args = rospy.myargv()[1:]
    bool_topic=args[0]
    drive_topic=args[1]

    # start the node

    rospy.init_node('decision manager')
    
    #initialize the decision manager
    dm = Decision_Manager(bool_topic,drive_topic)

    rospy.spin()