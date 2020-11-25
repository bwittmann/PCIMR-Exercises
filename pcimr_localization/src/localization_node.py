#!/usr/bin/env python3

import rospy
import numpy as np
from threading import Lock

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LocalizationNode():
    
    def __init__(self):

        # Init member variables
        self.sim_lock = Lock()

        # Retriece parameters
        self.move_prob = [0.9, 0.04, 0.04, 0.0, 0.02]

        # Init publishers
        self.pub_robot_pos = rospy.Publisher('/robot_pos', Point, queue_size=10)
        self.pub_marker = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
        self.pub_occu_grid = rospy.Publisher('/visualization/robot_pos_array', OccupancyGrid, queue_size=10)

        # Init subscribers
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.cb_map)
        self.sub_sensor = rospy.Subscriber('/scan', LaserScan, self.cb_sensor)
        self.sub_move = rospy.Subscriber('/move', String, self.cb_move)

        # Setup messages for publishing
        self.msg_robot_pos = Point()
        self.msg_robot_pos.x = 2
        self.msg_robot_pos.y = 2
        self.msg_robot_pos.z = 0


        # Init instance variables
        self.map = np.array([])
        self.ranges = np.array([])
        self.action = ''
        
    def cb_map(self, msg):
        self.sim_lock.acquire()
        self.map = msg.data
        self.sim_lock.release()

    def cb_sensor(self, msg):
        self.sim_lock.acquire()
        self.ranges = msg.ranges
        self.sim_lock.release()

    def cb_move(self, msg):
        self.sim_lock.acquire()
        self.action = msg.data
        self.sim_lock.release()


    @staticmethod
    def map_init(map):
        pass

    def run(self, rate=1):

        while not rospy.is_shutdown():

            #self.pub_robot_pos.publish(self.msg_robot_pos)
            #rospy.loginfo(self.map)
            rospy.loginfo(self.ranges)
            #rospy.loginfo(self.action)

            rospy.sleep(1/rate)


if __name__ == "__main__":
    rospy.init_node('localization_node')

    localization_node = LocalizationNode()
    localization_node.run()


