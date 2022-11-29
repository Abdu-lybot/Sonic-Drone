#!/usr/bin/env python

import random
import time
import sys
import rospy
import tf
from circle import MavController
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
#from termcolor import colored



class DipteraAutoFly:

    def __init__(self):
        # rospy.init_node("Diptera_Auto_node")
        rospy.Subscriber("/Diptera/Obstacle", String, self.cb_obst_status, queue_size=1)
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.cb_pose, queue_size=1)
        # rospy.Subscriber("/mavros/battery, ")
        self.pi_2 = 3.14 / 2
        # self.path_status = String()
        self.move = "MOVE"
        self.blocked_left = "LB"
        self.blocked_middle = "MB"
        self.blocked_right = "RB"
        self.mission_steps = 30
        self.ini_step = 0
        self.step_time = 6
        self.fixed_altitude = 0.5
        self.front_dir = 0
        self.right_dir = -1 * self.pi_2
        self.left_dir = self.pi_2
        self.back_dir = 2 * self.pi_2
        self.fixed_x = 0
        self.fixed_y = 0
        self.securex = 0
        self.securey = 0
        self.path_status = str

    def cb_pose(self, msg):
        self.local = msg
        self.localx = msg.pose.position.x
        self.localy = msg.pose.position.y
        self.localz = msg.pose.position.z
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]
        # print("roll is %f"  %roll)
        # print("pitch is %f" %pitch)
        # print ("yaw is %f" %self.yaw)
        # print (self.local)
        # print (self.localx)
        # print (self.localy)
        # print (self.localz)

    def cb_obst_status(self, msg):
        self.path_status = msg.data
        # print (path_status)
        # r = rospy.Rate(0.5)
        rospy.sleep(0.5)
        self.flybase(self.path_status)

    def flybase(self, path_status):
        # print (status)
        if self.move == path_status:
            self.random_goto("forward")
        elif self.blocked_right == path_status:
            self.random_goto("turnL")
        elif self.blocked_middle == path_status:
            self.random_goto("turnB")
        elif self.blocked_left == path_status:
            self.random_goto("turnR")

    def mission_flight_time(self):
        self.ini_step = self.ini_step + 1
        return self.ini_step

    def clean_shutdown(self):
        ''' Stop robot when shutting down '''
        rospy.loginfo("System is shutting down. Stopping robot...")
        print("Time max Reached ..Landing")
        Move.land()
        time.sleep(5)
        Move.disarm()
        rospy.signal_shutdown("times up")

    def random_goto(self, decision):
        updated_x = 0
        updated_y = 0
        updated_z = 0
        flyingtime = self.mission_flight_time()
        print ("flying time is" ,flyingtime)
        if flyingtime == self.mission_steps:
            rospy.on_shutdown(self.clean_shutdown())
        raw_x = self.localx
        raw_y = self.localy
        self.fixed_x, self.fixed_y = self.vslam_security(raw_x, raw_y)
        self.updated_x = self.fixed_x
        self.updated_y = self.fixed_y
        if decision == "forward" :
            # self.waypoint(self.pitch_heading, self.roll_heading, self.altitude_heading, self.angul_pitch,self.angul_roll, 0)
            if (0.7853 < self.yaw < 2.3561):   #front
                self.updated_y = self.fixed_y + 1
                print("front in +Y dir, old %f ,new %f" % (self.fixed_y, self.updated_y))
                Move.goto_xyz_rpy(self.fixed_x, self.updated_y, self.fixed_altitude, 0, 0, self.front_dir)
            elif (-0.7853 < self.yaw < 0.7853):   #left
                self.updated_x = self.fixed_x + 1
                print("front in +X dir, old %f ,new %f" % (self.fixed_x, self.updated_x))
                Move.goto_xyz_rpy(self.updated_x, self.fixed_y, self.fixed_altitude, 0, 0, self.left_dir)
            elif (-2.3561 < self.yaw < -0.7853):    #back
                self.updated_y = self.fixed_y - 1
                print("front in -Y dir, old %f ,new %f" % (self.fixed_y, self.updated_y))
                Move.goto_xyz_rpy(self.fixed_x, self.updated_y, self.fixed_altitude, 0, 0, self.back_dir)
            elif (-3.14 < self.yaw < -2.356) or (2.35 < self.yaw < 3.14):   #right
                self.updated_x = self.fixed_x - 1
                print("front in -X dir, old %f ,new %f" % (self.fixed_x, self.updated_x))
                Move.goto_xyz_rpy(self.updated_x, self.fixed_y, self.fixed_altitude, 0, 0, self.right_dir)
            print(decision)
            time.sleep(4)
            #self.dynamic_obs(self.step_time)
        elif decision == "turnL":
            left_angle = self.yaw
            Move.goto_xyz_rpy(self.fixed_x, self.fixed_y, self.fixed_altitude, 0, 0, left_angle)
            print("Obstacle on Right --> Turning Left", left_angle)
            time.sleep(self.step_time)
        elif decision == "turnB":
            back_angle = self.yaw + self.pi_2
            Move.goto_xyz_rpy(self.fixed_x, self.fixed_y, self.fixed_altitude, 0, 0, back_angle)
            print("Obstacle on Middle --> Turning Back", back_angle)
            time.sleep(self.step_time)
        elif decision == "turnR":
            right_angle = self.yaw - 2*self.pi_2
            Move.goto_xyz_rpy(self.fixed_x, self.fixed_y, self.fixed_altitude, 0, 0, right_angle)
            print("Obstacle on Left --> Turning Right", right_angle)
            time.sleep(self.step_time)
        self.securex = self.updated_x
        self.securey = self.updated_y

    def vslam_security(self,x ,y):
        if (self.securex + 1 >= x >= self.securex - 1) & (self.securey + 1 >= y >= self.securey - 1):
            print("X & Y successfuly aquired")
            return x, y
        else:
            print("SLAM ERROR, Diptera security WARNING")
            time.sleep(0.3)
            raw_x = self.localx
            raw_y = self.localy
            x, y = self.vslam_security(raw_x, raw_y)
	    #Move.goto_xyz_rpy(self.updated_x, self.updated_y, self.fixed_altitude ,0 , 0, self.yaw - 90)
            return x, y

    def dynamic_obs(self, time):
        start = time.now
        while time.now < start + 5:
            if self.path_status == self.move:
                #time.sleep(0.2)
                continue
            else:
                #TODO: check that localx and localy are fine
                self.updated_x = self.localx
                self.updated_y = self.localy
                #Move.goto_xyz_rpy(self.updated_x, self.updated_y, self.fixed_altitude ,0 , 0, self.yaw - 90)
                self.clean_shutdown()
        return x, y


    def obst_status(self):
        rospy.spin()
        #self.flybase()
        print(self.path_status)


if __name__ == '__main__':
    Move = MavController()
    rospy.sleep(1)
    Move.takeoff(1)
    print("VENGA! Takeoff")
    rospy.sleep(4)
    fly = DipteraAutoFly()
    fly.obst_status()
