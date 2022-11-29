
#!/usr/bin/env python

import random
import time
import sys
import rospy
import tf
from circle import MavController
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class DipteraAutoFly:

    def __init__(self):
        #rospy.init_node("Diptera_Auto_node")
        rospy.Subscriber("/Diptera/Obstacle", String, self.cb_obst_status, queue_size=1)
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.cb_pose, queue_size=1)
        # rospy.Subscriber("/mavros/battery, ")
        self.pi_2 = 3.14 / 2
        # self.path_status = String()
        self.move = "MOVE"
        self.blocked = "BLOCKED"
        self.mission_steps = 30
        self.ini_step = 0
        self.step_time = 5
        self.fixed_altitude = 0.9

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
        path_status = msg.data
        # print (path_status)
        # r = rospy.Rate(0.5)
        rospy.sleep(0.5)
        self.flybase(path_status)

    def flybase(self, path_status):
        # print (status)
        if self.move == path_status:
            self.random_goto("forward")
        elif self.blocked == path_status:
            self.random_goto("turn")

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
        if decision == "forward":
            # self.waypoint(self.pitch_heading, self.roll_heading, self.altitude_heading, self.angul_pitch,self.angul_roll, 0)
            if (0.7853 < self.yaw < 2.3561):       #front
                updated_y = self.localy + 1
                print("front in +Y dir, old %f ,new %f" % (self.localy, updated_y))
                Move.goto_xyz_rpy(self.localx, updated_y, self.fixed_altitude, 0, 0, 0)
            elif (-0.7853 < self.yaw < 0.7853):    #right
                updated_x = self.localx + 1
                print("front in +X dir, old %f ,new %f" % (self.localx, updated_x))
                Move.goto_xyz_rpy(updated_x, self.localy, self.fixed_altitude, 0, 0, -1.57)
            elif (-2.3561 < self.yaw < -0.7853):    #back
                updated_y = self.localy - 1
                print("front in -Y dir, old %f ,new %f" % (self.localy, updated_y))
                Move.goto_xyz_rpy(self.localx, updated_y, self.fixed_altitude, 0, 0, 3.14)
            elif (-3.14 < self.yaw < -2.356) or (2.35 < self.yaw < 3.14):                    #left
                updated_x = self.localx - 1
                print("front in -X dir, old %f ,new %f" % (self.localx, updated_x))
                Move.goto_xyz_rpy(updated_x, self.localy, self.fixed_altitude, 0, 0, 1.57)
            print(decision)
            time.sleep(self.step_time)
        elif decision == "turn":
            # direction = []
            front_dir = 0
            left_dir = self.pi_2
            right_dir = -1 * self.pi_2
            back_dir = 2 * self.pi_2
            direction = ["t_front", "t_left", "t_right", "t_back"]
            rand_choice = random.randint(0, 3)
            if direction[rand_choice] == "t_front":
                # self.waypoint(0, 0.0, self.altitude_heading, 0, 0, forw_dir)
                Move.goto_xyz_rpy(self.localx, self.localy, self.fixed_altitude, 0, 0, 0)
                print("%s from angel %f" %(direction[rand_choice],self.yaw))
                #direction[rand_choice] = None
                #time.sleep(self.step_time)
            elif direction[rand_choice] == "t_left":
                # self.waypoint(0, 0.0, self.altitude_heading, 0, 0, left_dir)
                Move.goto_xyz_rpy(self.localx, self.localy, self.fixed_altitude, 0, 0, 1.57)
                print("%s from angel %f" % (direction[rand_choice], self.yaw))
                #direction[rand_choice] = None
                #time.sleep(self.step_time)
            elif direction[rand_choice] == "t_right":
                # self.waypoint(0, 0.0, self.altitude_heading, 0, 0, right_dir)
                Move.goto_xyz_rpy(self.localx, self.localy, self.fixed_altitude, 0, 0, -1.57)
                print("%s from angel %f" % (direction[rand_choice], self.yaw))
                #direction[rand_choice] = None
                #time.sleep(self.step_time)
            elif direction[rand_choice] == "t_back":
                # self.waypoint(0, 0.0, self.altitude_heading, 0, 0, back_dir)
                Move.goto_xyz_rpy(self.localx, self.localy, self.fixed_altitude, 0, 0, 3.14)
                print("%s from angel %f" % (direction[rand_choice], self.yaw))
                #direction[rand_choice] = None
            time.sleep(self.step_time)

    def obst_status(self):
        rospy.spin()
        # self.flybase()
        # print(self.path_status)


if __name__ == '__main__':
    Move = MavController()
    rospy.sleep(1)
    Move.takeoff(0.9)
    print("VENGA! Takeoff")
    time.sleep(4)
    #print("going first angle 3.14")
    #Move.goto_xyz_rpy(0, 0, 1, 0, 0, 3.14)
    #print("going second angle")
    #time.sleep(4)
    #Move.goto_xyz_rpy(0, 0, 1, 0, 0, 0)
    #time.sleep(7)
    #Move.land()
    fly = DipteraAutoFly()
    fly.obst_status()
