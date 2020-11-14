#!/usr/bin/env python3

import rospy
from threading import Lock

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from pcimr_simulation.srv import InitPos


class PlannerNode():
    """
    Node used for guiding the robot from its initial position to the goal.
    """
    
    def __init__(self):
        """
        Initialize a instance of the PlannerNode class, including the node and its 
        publishers and subscribers. 
        """
        # Init member variables
        self.sim_lock = Lock()

        # Init publishers
        self.pub_move = rospy.Publisher('/move', String, queue_size=10)

        # Init subscribers
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.cb_scan)
        self.sub_robot_position = rospy.Subscriber('/robot_pos', Point, self.cb_robot_position)

        # Init messages for publishing
        self.msg_robot_move = String()

        # Init variables for position and range measurements
        self.pos = (0, 0, 0)
        self.meas = [0, 0, 0, 0]



    def cb_robot_position(self, msg):
        """
        Callback function for retrieving current position of the robot.

        @param msg: Message containing the current robot position (x, y, z)
        """
        self.sim_lock.acquire()
        rospy.loginfo("Current Robot Pos: x = {},y = {}".format(msg.x, msg.y))
        self.pos = msg.x, msg.y, msg.z
        self.sim_lock.release()

    def cb_scan(self, msg):
        """
        Callback function for retrieving values of the robots laser scan.

        @param msg: Message containing information and data of the laser scan (angle, minimum range,
        maximum range, scan time, etc.)
        """
        self.sim_lock.acquire()
        self.meas = msg.ranges
        self.sim_lock.release()
        
    def init_pos_client(self, x, y):
        """
        Client for setting the initial position of the robot via the service /init_pos which is
        implemented in the node simple_sim_node.

        @param x: initial x-position of the robot
        @param y: initial y-position of the robot
        """
        rospy.wait_for_service('init_pos')
        try:
            init_pos = rospy.ServiceProxy('init_pos', InitPos)
            resp = init_pos(x, y)
            rospy.loginfo(str(resp.success) + " -> Robot position has been set to (2,0)")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    def run(self, rate=1):
        '''
        Function that leads the robot towards the goal based on a simple logic by publishing orders
        to the /move topic

        @param rate: used to time the publishing of the orders based on 1/rate
        '''
        while not rospy.is_shutdown():

            # If it is possible the robot should go north.
            if self.meas[2] > 1:
                self.msg_robot_move.data = "N"
                self.pub_move.publish(self.msg_robot_move)
                rospy.sleep(1/rate)
                continue
            
            # If it os possible the robot should go right.
            if self.meas[3] > 1:
                self.msg_robot_move.data = "E"
                self.pub_move.publish(self.msg_robot_move)
                rospy.sleep(1/rate)
                continue
            
            # Terminate the node when the goal position has been reached.
            if self.pos[0] == 16 and self.pos[1] == 12:
                rospy.loginfo("The goal has been reached!")
                break



if __name__ == "__main__":
    rospy.init_node('simple_planner_node')

    simple_planner_node = PlannerNode()

    # Call the service /init_pos to set the initial robot position. 
    simple_planner_node.init_pos_client(2,0)
    simple_planner_node.run()