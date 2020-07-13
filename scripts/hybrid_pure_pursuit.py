#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from f1tenth_gym_ros.msg import StampedBool
from f1tenth_gym_ros.msg import GoalPoint
import numpy as np


class Hybrid_Pure_Pursuit:
    def __init__(self,disparity_topic,pure_pursuit_topic,opp_odom_topic,drive_topic,own_odom):
        print(disparity_topic,pure_pursuit_topic,opp_odom_topic,drive_topic,own_odom)

        # subscriber to messages published by the disparity extender
        self.disparity_sub=Subscriber(disparity_topic,AckermannDriveStamped,queue_size=10)

        # subscriber to messages pbulised by the pure_pursuit node
        self.pure_pursuit_sub=Subscriber(pure_pursuit_topic,AckermannDriveStamped,queue_size=10)

        # subscriber to the opponents odom, since we get it for free (this isn't ideal but eh)
        self.opponent_odometry_sub=Subscriber(opp_odom_topic,Odometry,queue_size=10)

        self.own_odometry=Subscriber(own_odom,Odometry,queue_size=10)

        self.sub = ApproximateTimeSynchronizer([self.disparity_sub,self.pure_pursuit_sub,self.opponent_odometry_sub,self.own_odometry], queue_size = 10, slop = 0.05)
        
        # debug visualization of marker array
        self.goal_pub = rospy.Publisher('pure_pursuit_goal_point', MarkerArray, queue_size="1")

        self.drive_publisher=rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=1)

        # switching threshold 
        self.switch_distance_threshold = 2.0
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)


    def master_callback(self,disparity_msg,pure_pursuit_msg,odom,own_odom):
        pt = np.asarray([own_odom.pose.pose.position.x,own_odom.pose.pose.position.y]).reshape((1,2))
        opp_pt = np.asarray([odom.pose.pose.position.x,odom.pose.pose.position.y]).reshape((1,2))

        distance = np.linalg.norm(opp_pt-pt)

        # Depending on the distance of the goal point and the car in front of you
        # select which message to route through

        disparity_msg.header.stamp=rospy.Time.now()
        pure_pursuit_msg.header.stamp=rospy.Time.now()

        rospy.loginfo("distance: "+str(distance))
        if(distance< self.switch_distance_threshold):

            disparity_msg.drive.speed = self.set_speed(disparity_msg.drive.steering_angle,odom.twist.twist.linear.x)
            

            self.drive_publisher.publish(disparity_msg)
        else:
            self.drive_publisher.publish(pure_pursuit_msg)
        self.visualize_point(pt,self.goal_pub)




    def set_speed(self,angle,opp_speed):
        angle = abs(angle)
        if(angle<0.0572665):
            return max(opp_speed*2.0,4.6)
        elif (angle<0.174533):
            return max(opp_speed*2.0,4.4)
        elif(angle < 0.261799):
            return max(opp_speed*2.0,3.8) 
        elif(angle< 0.349066):
            return 2.8 
        else:
            return opp_speed


    def visualize_point(self,pts,publisher,frame='/map',r=1.0,g=0.0,b=1.0):
        # create a marker array
        markerArray = MarkerArray()

        idx = np.random.randint(0,len(pts))
        pt = pts[idx]

        x = float(pt[0])
        y = float(pt[1])
		
        marker = Marker()
        marker.header.frame_id = frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        markerArray.markers.append(marker)
        publisher.publish(markerArray)

if __name__ == '__main__':
    rospy.init_node('hybrid_pure_pursuit')
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    

    disparity_topic=args[0]
    pure_pursuit_topic=args[1]
    opp_odom_topic=args[2]
    drive_topic=args[3]
    own_odom=args[4]

    C = Hybrid_Pure_Pursuit(disparity_topic,pure_pursuit_topic,opp_odom_topic,drive_topic, own_odom)  
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        r.sleep()