#!/usr/bin/env python

import time
import random
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf
from circle import MavController
from geometry_msgs.msg import PoseStamped

class AutoFly:

    def __init__(self):
        
        self.pi_2 = 3.14 / 2
        self.mission_steps = 20
        self.ini_step = 0
        self.step_time = 4
        self.fixed_altitude = 1.1
        self.laser = float
        self.minimum = float
        self.angle = int
        self.min_obs_dis = 2.5
        self.move = "MOVE"
        self.blocked = "BLOCKED"
        self.obs_msg = "Obstacle ahead"
        self.no_obs_msg = "Clear path ahead"
        # self.rate = rospy.Rate(1)
        self.lastmsg = str
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

    def cb_lzr(self, msg):
        minimum = float
        for idx in range(120, 220, 4):
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
            self.path_status = self.blocked
            self.flybase()
            print("obstacle ahead")
        else:
            self.path_status = self.move
            self.flybase()
            print("area clear")

    def flybase(self):
        # print (status)
        if self.move == self.path_status:
            self.lastmsg = "forward"
            self.random_goto()
        elif self.blocked == self.path_status:
            self.lastmsg = "turn"
            self.random_goto()

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

    def random_goto(self):
        updated_x = 0
        updated_y = 0
        updated_z = 0
        flyingtime = self.mission_flight_time()
        print("last msg" ,self.lastmsg)
        print ("flying time is" ,flyingtime)
        if flyingtime == self.mission_steps:
            rospy.on_shutdown(self.clean_shutdown())
        if self.lastmsg == "forward":
            # self.waypoint(self.pitch_heading, self.roll_heading, self.altitude_heading, self.angul_pitch,self.angul_roll, 0)
            if (0.7853 < self.yaw < 2.3561):
                updated_y = self.localy + 1
                #print("front in +Y dir, old %f ,new %f" % (self.localy, updated_y))
                #Move.goto_xyz_rpy(updated_x, self.localy, self.fixed_altitude, 0, 0, self.yaw)
            elif (-0.7853 < self.yaw < 0.7853):
                updated_x = self.local + 1
                #print("front in +X dir, old %f ,new %f" % (self.localx, updated_x))
                #Move.goto_xyz_rpy(self.localx, updated_y, self.fixed_altitude, 0, 0, self.yaw)
            elif (-2.3561 < self.yaw < -0.7853):
                updated_y = self.localy - 1
                #print("front in -Y dir, old %f ,new %f" % (self.localy, updated_y))
                #Move.goto_xyz_rpy(updated_x, self.localy, self.fixed_altitude, 0, 0, self.yaw)
            elif (-3.14 < self.yaw < -2.356) or (2.35 < self.yaw < 3.14):
                updated_x = self.localx - 1
                #print("front in -X dir, old %f ,new %f" % (self.localx, updated_x))
                #Move.goto_xyz_rpy(self.localx, updated_y, self.fixed_altitude, 0, 0, self.yaw)
            print(self.lastmsg)
            time.sleep(self.step_time)
        elif self.lastmsg == "turn":
            # direction = []
            front_dir = 0
            left_dir = self.pi_2
            right_dir = -1* self.pi_2
            back_dir = 2 * self.pi_2
            direction = ["t_front", "t_left", "t_right", "t_back"]
            rand_choice = random.randint(0, 3)
            if direction[rand_choice] == "t_front":
                # self.waypoint(0, 0.0, self.altitude_heading, 0, 0, forw_dir)
                #Move.goto_xyz_rpy(self.localx, self.localy, self.fixed_altitude, 0, 0, front_dir)
                #print("%s from angel %f" %(direction[rand_choice],self.yaw))
                #direction[rand_choice] = None
                time.sleep(self.step_time)
            elif direction[rand_choice] == "t_left":
                # self.waypoint(0, 0.0, self.altitude_heading, 0, 0, left_dir)
                #Move.goto_xyz_rpy(self.localx, self.localy, self.fixed_altitude, 0, 0, left_dir)
                #print("%s from angel %f" % (direction[rand_choice], self.yaw))
                #direction[rand_choice] = None
                time.sleep(self.step_time)
            elif direction[rand_choice] == "t_right":
                # self.waypoint(0, 0.0, self.altitude_heading, 0, 0, right_dir)
                #Move.goto_xyz_rpy(self.localx, self.localy, self.fixed_altitude, 0, 0, right_dir)
                #print("%s from angel %f" % (direction[rand_choice], self.yaw))
                #direction[rand_choice] = None
                time.sleep(self.step_time)
            elif direction[rand_choice] == "t_back":
                # self.waypoint(0, 0.0, self.altitude_heading, 0, 0, back_dir)
                #Move.goto_xyz_rpy(self.localx, self.localy, self.fixed_altitude, 0, 0, back_dir)
                #print("%s from angel %f" % (direction[rand_choice], self.yaw))
                #direction[rand_choice] = None
                time.sleep(self.step_time)



    def listiner(self):
        rospy.init_node("Diptera_Obstacle_detection")
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.cb_pose, queue_size=1)
        rospy.Subscriber("/astra/scan", LaserScan, self.cb_lzr, queue_size=1)
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.cb_pose, queue_size=1)
        rospy.spin()


if __name__ == '__main__':

    #Move = MavController()
    #time.sleep(1)
    #Move.takeoff(1)
    print("VENGA! Takeoff")
    time.sleep(5)

    cld = AutoFly()
    cld.listiner()
    cld.min_dis()
