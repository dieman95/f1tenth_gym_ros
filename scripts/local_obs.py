#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
# from tf2_ros import transform_broadcaster
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
# from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class local_obs:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = 'scan'
        odom_topic = 'odom'
        obstacle_topic = 'obs_loc'

        # Initialize publishers and subscribers
        self.lidar_sub = Subscriber(lidarscan_topic, LaserScan, queue_size=1)
        self.odom_sub = Subscriber(odom_topic, Odometry, queue_size=1)
        # self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        # self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.lidar_callback, queue_size=1)
        self.pub = rospy.Publisher(obstacle_topic, MarkerArray, queue_size="1")

         #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.lidar_sub,self.odom_sub], queue_size = 10, slop = 0.02)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)
    

    def master_callback(self, lidarD, odomD):
        ################### 1) Get lidar data ################################
        ranges = lidarD.ranges
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

        les = self.obsCoordinates_mark(obstacles,ranges) # Obstacles for markers
        les_car = self.obsCoordinates_car(obstacles,ranges) # Obstacles for car (rtreach)   
        
        ##########  2) Get location and orientation of car  ############################
        car = odomD.pose.pose
        (roll, pitch, yaw) = euler_from_quaternion([car.orientation.x, car.orientation.y, car.orientation.z, car.orientation.w])
        x = car.position.x
        y = car.position.y
        car_pos = [x,y,roll,pitch,yaw]
        print('Car driving at angle  ' + str(yaw))


        ########## 3) Create obstacles as boxes ###########################

        obs_intervals = self.obsZono_local(les_car) # vertices of rectangle obstacles
        map_obs_intervals = self.obsZono_gen(les,car_pos) # vertices of rectangle obstacles fixed coordinates (map)
        # print(len(obs_intervals))
        # print(len(map_obs_intervals))

        ########## 4) Combine data to creat markers  ###########################
        obs_MarkerArray = self.createMakers(les, car_pos)
        # print(obs_MarkerArray)
        # Publish line markers
        self.pub.publish(obs_MarkerArray)

        return

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

    # Find any lidar points less than the min distance that the car may collide with in the next time step
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

# Could create markers based on vehicle's current position, but 
# that defeats the purpose of creating objects wrt obstacle position, 
# so visualization in a different function

# Define interval coordinates for each object
    def obsCoordinates_mark(self,obstacles,ranges):
        nob = len(obstacles)
        les = [] # list of all points of all obstacle (list of lists of coordinates x-y)
        for ii in range(nob):
            temp = obstacles[ii]
            pObs = [] # points to create straight line within obstacle (list of coordinates x-y)
            tlp = np.arange(temp[0],temp[1],4)
            for i in tlp:
                angle = -2.356 + i*0.25 # degrees
                angRad = angle*math.pi/180
                dist = ranges[i]
                xyCord = [dist, angRad]
                pObs.append(xyCord)
            les.append(pObs)
        return les

    # Define interval coordinates for each object
    def obsCoordinates_car(self,obstacles,ranges):
        nob = len(obstacles)
        les = [] # list of all points of all obstacle (list of lists of coordinates x-y)
        for ii in range(nob):
            temp = obstacles[ii]
            pObs = [] # points to create straight line within obstacle (list of coordinates x-y)
            tlp = np.arange(temp[0],temp[1],4)
            for i in tlp:
                angle = -2.356 + i*0.25 # degrees
                angRad = angle*math.pi/180
                dist = ranges[i]
                xCord = dist*math.cos(angRad)
                yCord = dist*math.sin(angRad)
                xyCord = [xCord, yCord, angRad] # x,y,angle lidar point
                pObs.append(xyCord)
            les.append(pObs)
        return les

    # Create small boxes as obstacles (based on distances from the car as origin)
    def obsZono_local(self,les):
        boxes = []
        for obs in les:
            for i in range(len(obs)-2):
                ob_cur = obs[i]
                ob_next = obs[i+1]
                xmin = min(ob_cur[0], ob_next[0])
                xmax = min(ob_cur[0], ob_next[0])
                ymin = min(ob_cur[1], ob_next[1])
                ymax = min(ob_cur[1], ob_next[1])
                box = [xmin,xmax,ymin,ymax]
                boxes.append(box)
        return boxes

    # Create small boxes as obstacles (fixed coordinates, map)
    def obsZono_gen(self,les,car):
        boxes = []
        for obs in les:
            for i in range(len(obs)-2):
                ob_cur = obs[i]
                ob_next = obs[i+1]
                Ang1 = car[4] - ob_cur[1]
                Ang2 = car[4] - ob_next[1]
                ob_cur[1] = ob_cur[0]*math.sin(Ang1)   # y-position (first point)
                ob_cur[0] = ob_cur[0]*math.cos(Ang1)   # x-position (first point)
                ob_next[1] = ob_next[0]*math.sin(Ang2) # y-position (second point)
                ob_next[0] = ob_next[0]*math.cos(Ang2) # x-position (second point)    
                xmin = min(ob_cur[0], ob_next[0])
                xmax = min(ob_cur[0], ob_next[0])
                ymin = min(ob_cur[1], ob_next[1])
                ymax = min(ob_cur[1], ob_next[1])
                box = [xmin,xmax,ymin,ymax] # create box with vertices
                boxes.append(box) # Add to the list
        return boxes

# Create line makers based on the coordinates defined in obsCoordinates
    def createMakers(self,les,car):
        ml = len(les)
        rgb_color = [0, 1, 0]
        markerArray = MarkerArray()
        for i in range(ml):
            ob = les[i]
            # lobs = len(ob)
            line = Marker() # Create linked line
            line.header.frame_id = "/map"
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.1
            line.scale.y = 0.1
            line.scale.z = 0.1
            line.color.a = 1.0
            line.color.r = rgb_color[0]
            line.color.g = rgb_color[1]
            line.color.b = rgb_color[2]
            for k in ob:
                point1 = Marker()
                point1.header.frame_id = "/map"
                point1.type = Marker.POINTS
                point1.action = Marker.ADD
                point1.scale.x = 0.1
                point1.scale.y = 0.1
                point1.scale.z = 0.1
                point1.color.a = 1.0
                point1.color.r = rgb_color[0]
                point1.color.g = rgb_color[1]
                point1.color.b = rgb_color[2]
                point1.pose.orientation.w = car[4]
                newAng = car[4] - k[1]
                x_rel = k[0]*math.cos(newAng)
                y_rel = k[0]*math.sin(newAng)
                point1.pose.position.x = car[0] + x_rel
                point1.pose.position.y = car[1] + y_rel
                point1.pose.position.z = 0
                line.points.append(point1)
            
            markerArray.markers.append(line)

            rgb_color[0] += 0.05
            rgb_color[1] -= 0.05
            rgb_color[2] += 0.05
        return markerArray

    # def lidar_callback(self, data):
    #     """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
    #     """
    #     ranges = data.ranges
    #     proc_ranges,danger_idxs = self.preprocess_lidar(ranges)
    #     #Find closest point to LiDAR
    #     index = np.argmin(proc_ranges) #  proc_ranges.index(min(proc_ranges))
    #     min_distance = ranges[index]
    #     # Change min_distance based on current speed and how far we look into the future to analyze safety
    #     if min_distance > 2:
    #         print('We are safe MA FRIENDS')
    #         return
    #     # Find number and size of obstacles
    #     obstacles = self.find_obs(proc_ranges,danger_idxs)
    #     if not obstacles:
    #         # self.pub.publish([Marker])
    #         return
        
    #     # print(obstacles)
    #     les = self.obsCoordinates(obstacles,ranges)
    #     # print('Coordinates of several points within obstacles')
    #     # print(les)
    #     # print('*********************************************************************************************')
    #     # self.pub.publish(small_obs)
    #     obs_MakerArray = self.createMakers(les)
    #     return
   
if __name__ == '__main__':
    rospy.init_node("localObs_node", anonymous=True)
    rfgs = local_obs()
    # rospy.sleep(0.1)
    rospy.spin()
