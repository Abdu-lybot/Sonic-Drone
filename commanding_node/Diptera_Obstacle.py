#!/usr/bin/env python

import time

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Cloudlaser:

    def __init__(self):

        self.laser = float
        self.minimum = float
        self.angle = int
        self.min_obs_dis = 2.5
        self.move = "MOVE"
        self.blocked = "BLOCKED"
        self.obs_msg = "Obstacle ahead"
        self.no_obs_msg = "Clear path ahead"
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
            self.publish_obs(self.blocked, self.obs_msg, self.angle, self.laser)
            print("obstacle ahead")
        else:
            self.publish_obs(self.move, self.no_obs_msg, self.angle, self.laser)
            print("area clear")

    def publish_obs(self, action, msg_info, angle, distance):
        obstinfo = rospy.Publisher('/Diptera/Obstacle/info', String, queue_size=1)
        obsmsg = rospy.Publisher('/Diptera/Obstacle', String, queue_size=1)
        # cont_msg = "%s at angle %s ,time %s" % msg ,%angle %rospy.get_time()
        msg_info_const = msg_info + ", distance: " + str(float(distance)) + ", angle: " + str(angle) + ", time: " + str(
            rospy.get_time())
        obstinfo.publish(msg_info_const)
        obsmsg.publish(action)

    def listiner(self):
        rospy.init_node("Diptera_Obstacle_detection")
        rospy.Subscriber("/astra/scan", LaserScan, self.cb_lzr, queue_size=1)
        rospy.spin()


# if __name__ == '__main__':
cld = Cloudlaser()
cld.listiner()
cld.min_dis()
