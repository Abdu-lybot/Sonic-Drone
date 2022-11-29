#!/usr/bin/env python

import time

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Cloudlaser:

    def __init__(self):

        self.laser = float
        self.minimum = float
        self.r_min_obs = float
        self.m_min_obs = float
        self.l_min_obs = float
        self.angle = int
        self.min_obs_dis = 3.5
        self.move = "MOVE"
        self.blocked_left = "LB"   #left is blocked
        self.blocked_middle = "MB"  #middle is blocked
        self.blocked_right = "RB"   #right is blocked
        self.left_obs_msg = "Obstacle ahead on Left"
        self.middle_obs_msg = "Obstacle ahead in the Middle"
        self.right_obs_msg = "Obstacle ahead on the Right"
        self.no_obs_msg = "Clear path ahead"
        # self.rate = rospy.Rate(1)

    def cb_lzr(self, msg):
        for idx in range(120, 220, 2):
            self.angle = idx
            if (120 <= idx <= 155):
                self.right_obs = msg.ranges[idx]
                self.r_min_obs = self.pointcloud_processing(self.right_obs)
                #print ("right")
            if (155 < idx < 185):
                self.middle_obs = msg.ranges[idx]
                self.m_min_obs = self.pointcloud_processing(self.middle_obs)
            if (185 <= idx <= 220):
                self.left_obs = msg.ranges[idx]
                self.l_min_obs = self.pointcloud_processing(self.left_obs)
                #print("left")
        #time.sleep(0.01)
        self.min_dis()

    def pointcloud_processing(self, point):
        minimum = float
        if point < minimum:
            minimum = point
        self.minimum = minimum
        #print (self.minimum)
        return minimum

    def min_dis(self):
        #print (self.minimum)
        if self.m_min_obs < self.min_obs_dis:
            self.publish_obs(self.blocked_middle, self.middle_obs_msg, self.angle, self.laser)
            print("obstacle ahead on Middle")
        elif self.r_min_obs <  self.min_obs_dis:
            self.publish_obs(self.blocked_right, self.right_obs_msg, self.angle, self.laser)
            print("obstacle ahead on RIGHT")
        elif self.l_min_obs < self.min_obs_dis:
            self.publish_obs(self.blocked_left, self.left_obs_msg, self.angle, self.laser)
            print("obstacle ahead on Left")
        else:
            self.publish_obs(self.move, self.no_obs_msg, self.angle, self.laser)
            print("area clear")

    def publish_obs(self, action, msg_info, angle, distance):
        obstinfo = rospy.Publisher('/Diptera/Obstacle/info', String, queue_size=1)
        obsmsg = rospy.Publisher('/Diptera/Obstacle', String, queue_size=1)
        # cont_msg = "%s at angle %s ,time %s" % msg ,%angle %rospy.get_time()
        #msg_info_const = msg_info + ", distance: " + str(float(distance)) + ", angle: " + str(angle) + ", time: " + str(
            #rospy.get_time())
        #obstinfo.publish(msg_info_const)
        obsmsg.publish(action)

    def listiner(self):
        rospy.init_node("Diptera_Obstacle_detection")
        rospy.Subscriber("/astra/scan", LaserScan, self.cb_lzr, queue_size=1)
        rospy.spin()


# if __name__ == '__main__':
cld = Cloudlaser()
cld.listiner()
cld.min_dis()
