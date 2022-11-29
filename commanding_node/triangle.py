#!/usr/bin/env python

# uncompyle6 version 3.7.4
# Python bytecode 2.7 (62211)
# Decompiled from: Python 2.7.17 (default, Sep 30 2020, 13:38:04) 
# [GCC 7.5.0]
# Embedded file name: /home/ubuntu/AdvanDiptera_VIO/src/commanding_node/triangle.py
# Compiled at: 2021-07-08 18:19:47


import rospy, tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
import math, time, numpy as np
from matplotlib import pyplot as plt
pi = math.pi
pi_2 = pi / 2.0

class MavController:
    """
    A simple object to help interface with mavros
    """

    def __init__(self):
        rospy.init_node('mav_control_node')
        rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback)
        self.cmd_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        self.rc_override = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()
        print 'Controller initizalized! Waiting 5 seconds for security reasons...'
        time.sleep(5)
        print 'Done!'
        self.pos_margin = 0.2
        self.yaw_margin = 15
        self.time_margin = 20
        self.x_orders = [
         0]
        self.y_orders = [0]
        self.z_orders = [0]
        self.yaw_orders = [0]
        self.x_list = [
         0]
        self.y_list = [0]

        

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose
        self.localx = self.pose.position.x
        self.localy = self.pose.position.y
        self.localz = self.pose.position.z
        quaternion = (
         self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = np.mod(euler[2] - pi_2 + pi, 2*pi) - pi
        self.yaw_deg = self.yaw * (180 / pi)

	self.current_ori = self.yaw_to_ori(self.yaw_deg)

    def yaw_to_ori(self,yaw):
	if -45 <= yaw <= 45:
            return "front"
        elif 45 <= yaw <= 135:
            return "left"
        elif -135 <= yaw <=-45:
            return "right"
        elif -180 <= yaw <= -135 or 135<=yaw<=180: 
            return "back"
	else:
	    return None

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose
        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya,wait=True):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)
        yaw_deg = ya * 180 / math.pi
	if wait:
            self.wait_until_reaching(x, y, z, yaw_deg)
            self.x_orders.append(x)
            self.y_orders.append(y)
            self.z_orders.append(z)
            self.yaw_orders.append(ya)

    def wait_until_reaching(self, x, y, z, yaw_deg):
        timer = 0
        while not (np.mod(self.yaw_deg - np.mod(yaw_deg - self.yaw_margin, 360), 360) <= np.mod(np.mod(yaw_deg + self.yaw_margin, 360) - np.mod(yaw_deg - self.yaw_margin, 360), 360) and x - self.pos_margin < self.localx < x + self.pos_margin and y - self.pos_margin < self.localy < y + self.pos_margin and z - self.pos_margin < self.localz < z + self.pos_margin):
            time.sleep(1)
	    if timer%2 != 0:
	        self.goto_xyz_rpy(x,y,z,0,0,yaw_deg*np.pi/180,False)
            timer += 1
            if timer == self.time_margin:
                break

        if timer == self.time_margin:
            print "Oups! The drone didn't reach the destination..."
        else:
            print 'Destination reached!'
        print 'Current pose is ' + str(self.localx) + ' ' + str(self.localy) + ' ' + str(self.localz) + ' ' + str(self.yaw_deg)
        print 'Desired pose was ' + str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + str(yaw_deg)
        time.sleep(1.5)

    def change_pos(self, x, y, z):
        last_yaw_order = self.yaw_orders[(-1)]
        self.goto_xyz_rpy(x, y, z, 0, 0, last_yaw_order)
        self.x_list.append(self.localx)
        self.y_list.append(self.localy)

    def change_ori(self, yaw):
        last_x_order = self.x_orders[(-1)]
        last_y_order = self.y_orders[(-1)]
        last_z_order = self.z_orders[(-1)]
        self.goto_xyz_rpy(last_x_order, last_y_order, last_z_order, 0, 0, yaw)
        time.sleep(2.5)
        self.x_list.append(self.localx)
        self.y_list.append(self.localy)

    def get_current_ori(self):
	return self.current_ori

    def final_plot(self):
        plt.plot(self.x_list, self.y_list, c='b')
        plt.plot(self.x_orders, self.y_orders, c='y')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Orders vs Real Movement')
        plt.savefig('path.png')

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz
        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz
        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        mode_resp = self.mode_service(custom_mode='4')
        self.arm()
        takeoff_resp = self.takeoff_service(altitude=height)
        return mode_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        self.final_plot()
        resp = self.mode_service(custom_mode='9')
        self.disarm()

comm = MavController()
print("Starting main")
time.sleep(5)
print("Takeoff!")
comm.takeoff(0.6)
comm.goto_xyz_rpy(0,0,0.6,0,0,0)
print("Let's wait!")
time.sleep(240)
print("Land")
comm.land()
time.sleep(5)
