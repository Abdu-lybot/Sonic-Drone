#!/usr/bin/env python


import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from pyquaternion import Quaternion
import time
import yaml
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float64, String

from Diptera_move_drone import Move_Drone as Basic_movement

class UltrasonicSensing():


    def __init__(self):


        rospy.init_node("Obstacle_detection_node")

        self.yamlpath = '/home/lybot/AdvanDiptera/src/commanding_node/params/obstacle_detection_params.yaml'
        with open(self.yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "min_distance_to_evit":
                 self.min_distance_to_evit = value



        self.state = str
        subscribed_topic_state= "/mavros/state"
        self.mavros_sub = rospy.Subscriber(subscribed_topic_state, State, self.cb_mavros_state)  # Not being used


        subscribed_topic_activity = ""     #write here the published topic name "activity"
        self.custom_activity_sub = rospy.Subscriber(subscribed_topic_activity, String, self.cb_activity)

        subscribed_topic_f= "/sonarTP_F"
        self.forward_sensor = rospy.Subscriber(subscribed_topic_f, Range, self.cb_forward_sensor)
        subscribed_topic_r= "/sonarTP_R"
        self.right_sensor = rospy.Subscriber(subscribed_topic_r, Range, self.cb_right_sensor)
        subscribed_topic_l= "/sonarTP_L"
        self.left_sensor = rospy.Subscriber(subscribed_topic_l, Range, self.cb_left_sensor)
        subscribed_topic_b= "/sonarTP_B"
        self.back_sensor = rospy.Subscriber(subscribed_topic_b, Range, self.cb_back_sensor)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)

        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)

        self.land_activity_sub = rospy.Publisher("/hover_status", String, self.cb_activity_land)
        self.hover_activity_sub = rospy.Publisher("/land_status", String, self.cb_activity_hover)


    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        while rospy.is_shutdown() is False: # While we don't shutdown it, do the loop

            time.sleep(0.1) # Rate 



    def cb_mavros_state(self, msg):
        self.mavros_state = msg.mode

    def cb_activity_land(self, msg):
        print("Received Custom Activity: ", msg.data)
            if msg.data == "LANDING":
            self.state = "LANDING"

    def cb_activity_hover(self, msg):
        print("Received Custom Activity: ", msg.data)
        if msg.data == "HOVER":
            self.state = "HOVER"


    def cb_forward_sensor(self, sense):
        if (self.state == "HOVER"):
            if sense.range < self.min_distance_to_evit:
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance
                diference = self.min_distance_to_evit - sense.range
                Basic_movement().moving_back(diference)
                rospy.logwarn("Avoiding Front Obstacle")

    def cb_right_sensor(self,sense):
        if (self.state == "HOVER"):
            if sense.range < self.min_distance_to_evit:
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance
                diference = self.min_distance_to_evit - sense.range

                Basic_movement().moving_left(diference)

                rospy.logwarn("Avoiding Right Obstacle") 

    def cb_left_sensor(self,sense):
        if (self.state == "HOVER"):
            if sense.range < self.min_distance_to_evit:
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance
                diference = self.min_distance_to_evit - sense.range

                Basic_movement().moving_right(diference)

                rospy.logwarn("Avoiding Left Obstacle")  

    def cb_back_sensor(self,sense):
        if (self.state == "HOVER"):
            if sense.range < self.min_distance_to_evit:
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance
                diference = self.min_distance_to_evit - sense.range

                Basic_movement().moving_forward(diference)

                rospy.logwarn("Avoiding Back Obstacle")

    def cb_down_sensor(self,sense):
        if (self.state == "HOVER"):
            if sense.range < self.min_distance_to_evit:
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance
                diference = self.min_distance_to_evit - sense.range

                Basic_movement().moving_up(diference)
 
                rospy.logwarn("Avoiding Down Obstacle")



if __name__ == '__main__':

    sensing = UltrasonicSensing()
    sensing.start()




