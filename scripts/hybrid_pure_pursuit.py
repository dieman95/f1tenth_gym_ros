#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from f1tenth_gym_ros.msg import StampedBool
from f1tenth_gym_ros.msg import GoalPoint
import numpy as np


class Hybrid_Pure_Pursuit:
    def __init__(self,disparity_topic,pure_pursuit_topic,opp_odom_topic,drive_topic,goal_point_topic):

        # subscriber to messages published by the disparity extender
        self.disparity_sub=Subscriber(disparity_topic,AckermannDriveStamped,queue_size=10)

        # subscriber to messages pbulised by the pure_pursuit node
        self.pure_pursuit_sub=Subscriber(pure_pursuit_topic,AckermannDriveStamped,queue_size=10)

        # subscriber to the opponents odom, since we get it for free (this isn't ideal but eh)
        self.opponent_odometry_sub=Subscriber(opp_odom_topic,Odometry,queue_size=10)

        self.pure_pursuit_goal_sub=Subscriber(goal_point_topic,GoalPoint,queue_size=10)

        self.sub = ApproximateTimeSynchronizer([self.disparity_sub,self.pure_pursuit_sub,self.opponent_odometry_sub,self.pure_pursuit_goal_sub], queue_size = 10, slop = 0.05)
        

        self.drive_publisher=rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=10)
        
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)


    def master_callback(self,disparity_msg,pure_pursuit_msg,odom,goal_point_topic):
        print(goal_point_topic)


if __name__ == '__main__':
    rospy.init_node('hybrid_pure_pursuit')
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    

    disparity_topic=args[0]
    pure_pursuit_topic=args[1]
    opp_odom_topic=args[2]
    drive_topic=args[3]
    gp_topic=args[4]

    C = Hybrid_Pure_Pursuit(disparity_topic,pure_pursuit_topic,opp_odom_topic,drive_topic,gp_topic)  
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        r.sleep()