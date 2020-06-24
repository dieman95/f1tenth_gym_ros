#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import numpy as np 

class reactive_follow_gap:
    def __init__(self):
        self.scan_debug=rospy.Publisher('scan_debug',LaserScan,queue_size=100)
        self.scan_sub=rospy.Subscriber('scan',LaserScan,self.laser_callback,queue_size=100)
        self.drive_pub=rospy.Publisher('drive',AckermannDriveStamped,queue_size=100)
        self.car_half_width = 0.25
        self.turn_clearance = 0.75
        rospy.spin()

    def laser_callback(self,msg):
        # get the ranges from the lidar scan messages
        ranges = msg.ranges
        behind_car=np.asarray(msg.ranges)

        # preprocess the ranges (eliminating things farther than 10.0)
        ranges = self.preprocess_ranges(ranges)

        
        #find the closest point index 
        min_index = ranges.argmin()
        val = ranges[min_index]

        # samples to bloat
        step_radians= 0.25*(np.pi/180)
        angle = np.arcsin(self.car_half_width/val)
        if not np.isnan(angle):
            num_samples = int(angle/step_radians)
            #print(val,int(angle/step_radians),num_samples)
            radius = int(num_samples/2)
            ranges[min_index-radius:min_index+radius+1] =0.0

            

            ranges,index = self.select_best_point(ranges)


            # publish the preprocessed data for debugging
            new_msg= msg
            new_msg.ranges = ranges 
            new_msg.header.stamp = rospy.Time.now()
            self.scan_debug.publish(new_msg)

            angle=(index-540)/4.0
            rad=(angle*np.pi)/180
            rad=self.threshold_angle(rad)

            behind_car_right=behind_car[0:180]
            behind_car_left=behind_car[901:]
            rad = self.adjust_turning_for_safety(behind_car_left,behind_car_right,rad)


            self.publish_speed_and_angle(rad)

        



    def preprocess_ranges(self,ranges):
        ranges = np.asarray(ranges)
        indices=np.where(ranges>=10.0)[0]
        ranges[indices]=10.0
        return ranges

    def adjust_turning_for_safety(self,left_distances,right_distances,angle):
        min_left=min(left_distances)
        min_right=min(right_distances)
        
        if min_left<=self.turn_clearance and angle>0.0:#.261799:
            rospy.logwarn("Too Close Left: "+str(min_left))
            angle=0.0
        elif min_right<=self.turn_clearance and angle<0.0:#-0.261799:
            rospy.logwarn("Too Close Right: "+str(min_right))
            angle=0.0
           
        else:
            angle=angle
        return angle
        

    def select_best_point(self,ranges):
        max_range=np.max(ranges)-1.0
        indices=np.where(ranges>=max_range)[0]
        ranges[:len(ranges)]=0
        indices = sorted(indices)
        index = (len(indices)-1)/2
        ranges[indices[index]] = max_range
        return ranges,indices[index]

    "Threshold the angle if it's larger than 35 degrees"
    def threshold_angle(self,angle):
        max_angle_radians=35*(np.pi/180)
        if angle<(-max_angle_radians):
            return -max_angle_radians
        elif angle>max_angle_radians:
            return max_angle_radians
        else:
            return angle

    """Function that publishes the speed and angle so that the car drives around the track"""
    def publish_speed_and_angle(self,angle):
        msg = AckermannDriveStamped()
        msg.header.stamp= rospy.Time.now()
        msg.drive.steering_angle = angle
        msg.drive.speed = 0.5 #right now I want constant speed
        self.drive_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("reactive_gap_following")
    rfg = reactive_follow_gap()