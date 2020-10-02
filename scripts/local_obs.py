#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class local_obs:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = 'scan'
        # obstacle_topic = 'obs_loc'

        self.lidar_sub = rospy.Subscriber( lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        # self.pub = rospy.Publisher(obstacle_topic, self.lidar_callback, queue_size="1")
 
   # lidar indexing starts from right to left
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)       """
        n = len(ranges)
        proc_ranges = [0]*n
        danger_idxs = []
        for i in range(n):
            proc_ranges[i] = (ranges[i] + ranges[i-1] + ranges[i-2])/3
            if ranges[i] > 2:
                proc_ranges[i] = 10
                danger_idxs.append(i)
            if ranges[i] == "nan":
                proc_ranges[i] =  max(proc_ranges[i-1], 0)
        return proc_ranges, danger_idxs


    def find_obs(self,proc_ranges,danger_idxs):
        n = len(danger_idxs)
        obstacles = []
        t1 = list(danger_idxs)
        t1.insert(0,t1[0])
        danger_idxs.append(danger_idxs[n-1])
        jumps = np.asarray(danger_idxs) - np.asarray(t1)
        jIdxs = np.where(jumps > 10)
        if len(jIdxs[0]) == 0:
            print('No danger... Keep the Autopilot going... zzZ')
            x1 = 0
            x2 = int(danger_idxs[n-1])
            return False
        else:
            jIdxs = list(jIdxs[0])
            for k in range(len(jIdxs)):
                if k==0:
                    x1 = 0
                    x2 = int(danger_idxs[jIdxs[k]-1])
                else:
                    x1 = int(danger_idxs[jIdxs[k-1]-1]+1)
                    x2 = int(danger_idxs[jIdxs[k]-1])
                obs = [x1,x2]
                obstacles.append(obs)

        return obstacles


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges,danger_idxs = self.preprocess_lidar(ranges)
        #Find closest point to LiDAR
        index = np.argmin(proc_ranges) #  proc_ranges.index(min(proc_ranges))
        min_distance = ranges[index]
        # Change min_distance based on current speed and how far we look into the future to analyze safety
        if min_distance > 2:
            print('We are safe MA FRIENDS')
            return
        # Find number and size of obstacles
        obstacles = self.find_obs(proc_ranges,danger_idxs)
        if not obstacles:
            # self.pub.publish([Marker])
            return
        
        # print(obstacles)
        small_obs = self.createMarkers(obstacles,ranges)
        print('Coordinates of several points within obstacles')
        print(small_obs)
        print('*********************************************************************************************')
        # self.pub.publish(small_obs)
        return

# Could create markers based on vehicle's current position, but 
# that defeats the purpose of creating objects wrt obstacle position, 
# so no visualization for now

    def createMarkers(self,obstacles,ranges):
        nob = len(obstacles)
        les = [] # list of all points of all obstacle (list of lists of coordinates x-y)
        for ii in range(nob):
            temp = obstacles[ii]
            pObs = [] # points to create straight line within obstacle (list of coordinates x-y)
            tlp = np.arange(temp[0],temp[1],20)
            for i in tlp:
                angle = i*0.25 # degrees
                angRad = angle*math.pi/180
                dist = ranges[i]
                xCord = dist*math.cos(angRad)
                yCord = dist*math.sin(angRad)
                xyCord = [xCord, yCord]
                print('Coordinates of the points in obstacle wrt to car position')
                print(xyCord)
                pObs.append(xyCord)
            les.append(pObs)
        return les

   
if __name__ == '__main__':
    rospy.init_node("localObs_node", anonymous=True)
    rfgs = local_obs()
    rospy.sleep(0.1)
    rospy.spin()
