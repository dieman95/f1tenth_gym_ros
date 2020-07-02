#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from f1tenth_gym_ros.msg import StampedBool
import numpy as np

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self,scan_topic,odom_topic,brake_bool_topic):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.brake_bool=rospy.Publisher(brake_bool_topic,StampedBool,queue_size=100)

        # Initialize subscribers
        self.scan_subscriber=Subscriber(scan_topic,LaserScan,queue_size=100)
        self.odom_subscriber=Subscriber(odom_topic,Odometry,queue_size=100)

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.scan_subscriber,self.odom_subscriber], queue_size = 100, slop = 0.020)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

        self.THRESHOLD=1.0

    def master_callback(self,scan_msg,odom_msg):
        
        degrees_to_radians= np.pi/180
        linear_velocity = odom_msg.twist.twist.linear.x
        angles_radian=(((np.arange(0,1080)-539)/4))*degrees_to_radians
        projections=np.cos(angles_radian)*linear_velocity
        projections=np.maximum(projections,0.00000001)
        
        # the velocity we get from odom is in the x direction  
        
        ranges = scan_msg.ranges

        # compute the time to collision
        ttc = ranges / projections
        
        # minimum ttc
        minimum_ttc = min(ttc)

        msg= StampedBool()
        msg.header.stamp=rospy.Time.now()
        if minimum_ttc < self.THRESHOLD:
            msg.data = True
            rospy.loginfo("Engage AEB")
            
        else:
            msg.data = False
        self.brake_bool.publish(msg)


def main():

    # get arguments from a launch file 
    args = rospy.myargv()[1:]
    scan_topic=args[0]
    odom_topic=args[1]
    brake_bool_topic = args[2]

    rospy.init_node('safety_node')
    sn = Safety(scan_topic,odom_topic,brake_bool_topic)
    rospy.spin()
if __name__ == '__main__':
    main()