#!/usr/bin/env python3

import rospy
import numpy as np
import datetime
from threading import Lock

from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LocalizationNode():
    '''
    Node that acts as a localization instance based on a discrete bayes filter.
    '''
    
    def __init__(self):
        '''
        Initialization of all necessary parameters, publishers, subscribers and instance variables.
        '''
        # Init member variables
        self.sim_lock = Lock()

        # Retriece parameters from parameter server
        self.move_prob = rospy.get_param('~robot_move_probabilities', [0.9, 0.04, 0.04, 0.0, 0.02])

        # Init publishers
        self.pub_robot_pos = rospy.Publisher('/robot_pos', Point, queue_size=10)
        self.pub_marker = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
        self.pub_occu_grid = rospy.Publisher('/robot_pos_map', OccupancyGrid, queue_size=10)

        # Init subscribers
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.cb_map)
        self.sub_sensor = rospy.Subscriber('/scan', LaserScan, self.cb_sensor)
        self.sub_move = rospy.Subscriber('/move', String, self.cb_move)

        # Setup messages for publishing
        self.msg_robot_pos = Point()
        self.msg_robot_pos.z = 0

        self.msg_marker = Marker()
        self.msg_marker.header.frame_id = "map"
        self.msg_marker.ns = "navigation"
        self.msg_marker.id = 0
        self.msg_marker.type = Marker.CUBE
        self.msg_marker.action = Marker.ADD
        self.msg_marker.scale.x = 1
        self.msg_marker.scale.y = 1
        self.msg_marker.scale.z = 0.2
        self.msg_marker.color.a = 1.0
        self.msg_marker.color.r = 0.0
        self.msg_marker.color.g = 1.0
        self.msg_marker.color.b = 0.0
        self.msg_marker.pose.orientation = Quaternion(0, 0, 0, 1)

        self.msg_occu_grid = OccupancyGrid()
        self.msg_occu_grid.header.frame_id = 'map'
        self.msg_occu_grid.header.seq = 0
        self.msg_occu_grid.info.map_load_time = rospy.Time.now()
        self.msg_occu_grid.info.resolution = 1
        self.msg_occu_grid.info.height = 20
        self.msg_occu_grid.info.width = 20

        # Init instance variables
        self.empty_map = np.array([], dtype=np.int16)
        self.map = np.array([], dtype=np.float32)
        self.sensor_expect = np.array([], dtype=np.int16)
        self.ranges = np.array([], dtype=np.float32)
        self.action = ''

        # Init constants
        self.MAP_WIDTH = 20
        self.MAP_HEIGHT = 20
        self.MEAS_UTIL = 4
    
    def cb_map(self, msg):
        '''
        Callback function for retrieving the empty map from the simple_sim_node.

        @param msg: Message containing a OccupancyGrid
        '''
        self.sim_lock.acquire()
        self.empty_map = np.array(msg.data)
        self.sim_lock.release()

    def cb_sensor(self, msg):
        '''
        Callback function for retrieving the measured distances to walls from the simple_sim_node.

        @param msg: Message containing a LaserScan
        '''
        self.sim_lock.acquire()
        self.ranges = np.array(msg.ranges)
        self.sim_lock.release()

    def cb_move(self, msg):
        '''
        Callback function for retrieving the move command (N, S, E, W) from the navigator_node.

        @param msg: Message containing a String
        '''
        self.sim_lock.acquire()
        self.action = str(msg.data)
        self.sim_lock.release()

    def bayes_init(self):
        '''
        Method for initializing the discrete bayes localization algorithm.
        If the current state of the robot is completely unknown, the probability distribution is uniform.
        A part of the initialization step is also to calculate the measurement likelihood (sensor model).
        '''
        # Uniform distribution
        self.map = np.copy(self.empty_map).astype(np.float32).reshape(self.MAP_HEIGHT,self.MAP_WIDTH)
        num_free_space = np.count_nonzero(self.empty_map==0)
        init_prob = 1/num_free_space
        self.map[self.map==0] = init_prob

        # Get measurement likelihood in form of self.sensor_expect tensor (sensor model)
        self.empty_map = self.empty_map.reshape(self.MAP_HEIGHT, self.MAP_WIDTH)
        self.sensor_expect = np.zeros((self.MAP_HEIGHT, self.MAP_WIDTH, self.MEAS_UTIL), dtype=np.int16)

        for col in range(self.empty_map.shape[0]):
            for row in range(self.empty_map.shape[1]):
                if self.empty_map[col, row] == 0:
                    counter_east = 0
                    counter_west = 0
                    counter_north = 0
                    counter_south = 0

                    r, c = row, col
                    try:
                        while self.empty_map[c, r] != 100:  # Since map is flipped, north and south are flipped
                            counter_east += 1
                            r += 1
                    except: 
                        pass

                    r, c = row, col
                    try:
                        while self.empty_map[c, r] != 100 and r >= 0:
                            counter_west += 1
                            r -= 1
                    except:
                        pass

                    r, c = row, col
                    while self.empty_map[c, r] != 100:
                        counter_north += 1
                        c += 1
                    
                    r, c = row, col
                    while self.empty_map[c, r] != 100:
                        counter_south += 1
                        c -= 1

                    self.sensor_expect[col, row] = [counter_south, counter_west, counter_north, counter_east]

        # Incorporate first measurement
        self.bayes_update()
        
    def bayes_prediction(self):
        '''
        Method for the prediction step in the discrete bayes localization algorithm.
        '''
        # Getting movement probabilities based on the current action and the motion model
        if self.action == 'N':
            prob_N = self.move_prob[0]
            prob_W = self.move_prob[1]
            prob_E = self.move_prob[2]
            prob_S = self.move_prob[3]
            prob_stay = self.move_prob[4]
        elif self.action == 'E':
            prob_N = self.move_prob[1]
            prob_W = self.move_prob[3]
            prob_E = self.move_prob[0]
            prob_S = self.move_prob[2]
            prob_stay = self.move_prob[4]
        elif self.action == 'S':
            prob_N = self.move_prob[3]
            prob_W = self.move_prob[2]
            prob_E = self.move_prob[1]
            prob_S = self.move_prob[0]
            prob_stay = self.move_prob[4]
        elif self.action == 'W':
            prob_N = self.move_prob[2]
            prob_W = self.move_prob[0]
            prob_E = self.move_prob[3]
            prob_S = self.move_prob[1]
            prob_stay = self.move_prob[4]

        current_map = np.copy(self.map)

        # Calculating the estimation based on the motion model for each grid cell
        for col in range(self.map.shape[0]):
            for row in range(self.map.shape[1]):
                if self.map[col, row] >= 0 and self.map[col, row] < 100:
                    # Include probability that robot does not move at all
                    self.map[col, row] *= prob_stay

                    if current_map[col+1, row] < 100:
                        self.map[col, row] += current_map[col+1, row] * prob_S     # N and S are switched 
                    else:
                        self.map[col, row] += current_map[col, row] * prob_N     # Robot hits wall and stays
        
                    if current_map[col-1, row] < 100:
                        self.map[col, row] += current_map[col-1, row] * prob_N
                    else:
                        self.map[col, row] += current_map[col, row] * prob_S

                    try:
                        if current_map[col, row+1] < 100:
                            self.map[col, row] += current_map[col, row+1] * prob_W 
                        else:
                            self.map[col, row] += current_map[col, row] * prob_E
                    except:
                        self.map[col, row] += self.map[col, row] * prob_E

                    try:
                        if current_map[col, row-1] < 100 and (row-1) >= 0:
                            self.map[col, row] += current_map[col, row-1] * prob_E
                        else:
                            self.map[col, row] += current_map[col, row] * prob_W
                    except:
                        self.map[col, row] += current_map[col, row] * prob_W

    def bayes_update(self):
        '''
        Method for the update step in the discrete bayes localization algorithm.
        '''
        # Multiply positions in the map with the probability that the measurement was observed at the specific position
        for col in range(self.map.shape[0]):
            for row in range(self.map.shape[1]):
                if sum(self.sensor_expect[col, row]) != 0:
                    diff = self.sensor_expect[col, row] - self.ranges
                    if max(diff) > 1:
                        self.map[col, row] = 0
                    else:
                        multiplyer = 1
                        for val in diff:
                            if val == 0:
                                multiplyer *= 0.8
                            else:
                                multiplyer *= 0.1
                        self.map[col, row] = self.map[col, row] * multiplyer

        # Normalization
        normal_mat = np.copy(self.map)
        normal_mat[(normal_mat==100) | (normal_mat==-1)] = 0
        normal_term = 1/np.sum(normal_mat)
        self.map[(self.map > 0) & (self.map < 100)] = self.map[(self.map > 0) & (self.map < 100)] * normal_term

        # Raise RuntimeError if normal_term is 'inf'. This happens when robot is kidnapped. 
        if normal_term == float('inf'):
            raise RuntimeError

        # Publish robot position ragarding believe and corresponding marker
        y,x = np.unravel_index(normal_mat.argmax(), normal_mat.shape)
        
        self.msg_marker.pose.position.x = x + 0.5
        self.msg_marker.pose.position.y = y + 0.5
        self.pub_marker.publish(self.msg_marker)

        self.msg_robot_pos.x = x
        self.msg_robot_pos.y = y
        self.pub_robot_pos.publish(self.msg_robot_pos)

    def publish_occu_grid(self):
        '''
        Method publishing a OccupancyGrid that contains the probabilities of the robot position.
        '''
        grid_to_publish = np.copy(self.map)
        grid_to_publish[(grid_to_publish > 0) & (grid_to_publish < 100)] = grid_to_publish[(grid_to_publish > 0) & (grid_to_publish < 100)] * 100
        self.msg_occu_grid.data = grid_to_publish.flatten().astype(int).tolist()
        self.pub_occu_grid.publish(self.msg_occu_grid)
        
    def bayes_main_loop(self):
        '''
        Main method of the discrete bayes localization algorithm combining the initialization step, the prediction step and the update step.
        '''
        # Wait in order to get self.empty_map updated via the subscription
        rospy.sleep(1)   

        # Initialize the algorithm
        self.bayes_init()

        while not rospy.is_shutdown():
            # Wait for the first published move message followed by the corresponding measurement.
            rospy.wait_for_message('/move', String)
            rospy.wait_for_message('/scan', LaserScan)

            if self.action != None:
                try:
                    # Prediction step
                    self.bayes_prediction()
                    # Update Step
                    self.bayes_update()
                except RuntimeError:    # If the robot looses track of its position it tries to initialize again (kidnapping)
                    self.bayes_init()

            self.publish_occu_grid()

            self.action = None  # Robot should not 'overrun' the goal if new one does not exist yet.


if __name__ == "__main__":
    # Create a ROS node named localization_node
    rospy.init_node('localization_node')

    localization_node = LocalizationNode()
    localization_node.bayes_main_loop()


