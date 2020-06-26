#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
import os 
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import rospkg 



class WaypointLogger():

    def __init__(self):

        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        # get the worldname from the parameter server
        world_name = rospy.get_param("/map_path").split('/')[-1].replace('.yaml','')
        #get the path for this paackage
        package_path=rospack.get_path('f1tenth_gym_ros')
        # get the pid to create "unique" filenames
        self.filename=package_path+'/waypoints/{}_{}.csv'.format(world_name,os.getpid())
        self.file = open(self.filename, 'w')

        self.waypoints=[[0,0]]

    def save_waypoint(self,data):
        pt = np.asarray([[data.pose.pose.position.x,data.pose.pose.position.y]])
        dist_arr = np.linalg.norm(np.asarray(self.waypoints)-pt,axis=-1)
        
        min_dist= np.min(dist_arr)
        if min_dist>0.14142135623730953:
            self.waypoints.append([data.pose.pose.position.x,data.pose.pose.position.y])
         
            print("x: {}, y: {}".format(data.pose.pose.position.x,data.pose.pose.position.y))
            self.file.write('%f, %f\n' % (data.pose.pose.position.x,
                                        data.pose.pose.position.y))

    def shutdown(self):
        self.file.close()
        print('Goodbye')
 
    def listener(self):
        rospy.init_node('waypoints_logger', anonymous=True)
        rospy.Subscriber('odom', Odometry, self.save_waypoint)
        rospy.spin()

if __name__ == '__main__':
    # create Waypoint Object
    wp = WaypointLogger()
    atexit.register(wp.shutdown)
    print('Saving waypoints...')
    try:
        wp.listener()
    except rospy.ROSInterruptException:
        pass
