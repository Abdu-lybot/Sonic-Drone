#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped, Twist
import time
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop

from Diptera_move_drone import Move_Drone as Basic_movement

class Landing:

    def __init__(self):

        self.yamlpath = '/home/lybot/AdvanDiptera/src/commanding_node/params/land_params.yaml'
        with open(self.yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "land_height":
                 self.land_height = value
              if key == "threshold":
                 self.threshold = value
              if key == "down_sensor_distance":
                 self.down_sensor_distance = value 
              if key == "final_land_threshold":
                 self.final_land_threshold = value
              if key == "landing_step_distance":
                 self.landing_step_distance = value 
 

        rospy.init_node("Landing_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback) 
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback) 
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber("/sonarTP_D", Range, self.cb_down_sensor)


    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw

        self.received_imu = True

    def q2yaw(self, q):

        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad 

    def local_pose_callback(self, msg): 

        self.local_pose = msg
        self.local_enu_position = msg

    def cb_down_sensor (self, msg):
        self.down_sensor_distance = msg.range 


    def start(self): # First with Offboard mode, put the drone in a certain alttitude. When the drone is in that position or lower, lands it. 

        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        for i in range(4)
            self.landing_activity_pub.publish("LANDING")
            time.sleep(0.1)

        Basic_movement().moving_to_z(self.land_height) # Probably not necesary here

        while (self.land_height - self.threshold) > self.local_pose.pose.position.z > (self.land_height + self.threshold) and (self.land_height - self.threshold) > self.down_sensor_distance > (self.land_height + self.threshold):
            Basic_movement().moving_to_z(self.land_height)
            time.sleep(0.2)

        while self.local_pose.pose.position.z > self.final_land_threshold and self.down_sensor_distance > self.final_land_threshold:
            Basic_movement().moving_down(self.landing_step_distance)
            time.sleep(1)
     
        for i in range (4):
            Basic_movement().moving_down(self.landing_step_distance)
            time.sleep(1)    
        
        
        


if __name__ == '__main__':
    land = Landing()
    land.start()
    time.sleep(8) 
        
