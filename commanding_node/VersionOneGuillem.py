#!/usr/bin/env python

import time

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LaserToDecision:

    def __init__(self):

	#rospy.init_node("Diptera_Obstacle_detection")
        rospy.Subscriber("/astra/scan", LaserScan, self.cb_lzr, queue_size=1)

        self.laser = float
        self.minimum = float
        self.angle = int
        self.min_obs_dis = 2
        self.move = "MOVE"
        self.turn = "TURN"
	self.wait = "WAIT"
	self.obstacle_state = self.wait  # By default
        # self.rate = rospy.Rate(1)

    def cb_lzr(self, msg):
        minimum = float
        for idx in range(120, 220, 2):
            # minimum = float
            self.laser = msg.ranges[idx]
            if self.laser < minimum:
                minimum = self.laser
        self.angle = idx
        self.minimum = minimum
        # time.sleep(1)
        self.min_dis()

    def min_dis(self):
        #print (self.minimum)
        if self.minimum < self.min_obs_dis:
            self.obstacle_state = self.turn
        else:
            self.obstacle_state = self.move

    def get_decision(self):
	return self.obstacle_state

