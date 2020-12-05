#!/usr/bin/env python3

import rospy
import numpy as np

import datetime

from threading import Lock

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData


class OccuGridMapAlgo():

    def __init__(self):

        # Init member variables
        self.sim_lock = Lock()
        
        # Init publishers
        self.pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # Init subscriptions
        self.sub_pos = rospy.Subscriber('/robot_pos', Point, self.cb_pos)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.cb_scan)

        # Setup messages for publishing
        self.msg_grid = OccupancyGrid()
        self.msg_grid.header.frame_id = 'map'
        self.msg_grid.info.map_load_time = rospy.Time.now()
        self.msg_grid.info.resolution = 1
        self.msg_grid.info.height = 20
        self.msg_grid.info.width = 20

        # Init instance variables
        self.ranges = None
        self.pos = None


    def cb_pos(self, msg):
        self.sim_lock.acquire()
        self.pos = (msg.x, msg.y)
        print(self.pos)
        self.sim_lock.release()

    def cb_scan(self, msg):
        self.sim_lock.acquire()
        self.ranges = msg.ranges
        print(self.ranges)
        self.sim_lock.release()

    def run(self):
        while not rospy.is_shutdown():
            rospy.wait_for_message('/scan', LaserScan)
            rospy.sleep(0.3)   # To ensure that also current robot_pos has been published
            
            print('loop')




if __name__ == "__main__":
    rospy.init_node('occu_grid_map_algo')

    occu_grid_map_node = OccuGridMapAlgo()
    occu_grid_map_node.run()
