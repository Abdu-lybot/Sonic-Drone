#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped, Twist
import time
import yaml
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float64, String

from Diptera_move_drone import Move_Drone as Basic_movement

class Lift_Off:

    def __init__(self):
        self.yamlpath = '/home/lybot/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as f:
    
           data = yaml.load(f)
           for key, value in data.items():
              if key == "takeoff_height":
                 self.takeoff_height = value
              if key == "Hover_distance":
                  self.hover_hight = value

        self.threshold = 0.3                      # Can be changed in params.yaml
        self.down_sensor_distance = 0
        self.current_heading = int
        self.flying_status = str

        rospy.init_node("Liftoff_hover_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.cb_imu)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)
        self.landing_activity_sub = rospy.Subscriber("/land_status", String, self.cb_landing_status)

        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.custom_activity_pub = rospy.Publisher("/hover_status", String, queue_size=10)

    def cb_imu(self, msg):
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

    def cb_local_pose(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg

    def constructCommandTOL(self, x, y, z, yaw, min_pitch = 0):
        command = CommandTOL()
        command.min_pitch = min_pitch
        command.latitude = x
        command.longitude = y
        command.altitude = z
        command.yaw = yaw
        return command

    def modechnge_takeoff(self):
        rospy.init_node("liftoff_node")
        if self.flightModeService(costume_mode='AUTO.TAKEOFF'):       # http://wiki.ros.org/mavros/CustomModes
            rospy.loginfo("succesfully changed mode to AUTO.TAKEOFF")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False

    def autotakeoff(self):
        if (self.takeoffService(self.constructCommandTOL(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.takeoff_height, self.current_heading))):
            rospy.loginfo("AdvanDiptera is Hovering")
            self.flying_status = "Drone_lifted"
            return True 
        else:
            rospy.loginfo("Failed to takeoff AdvanDiptera")
            return False

    def modechnge(self):
        rospy.init_node("offboard_node")
        if self.flightModeService(costume_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False

    def cb_down_sensor (self, msg):
        self.down_sensor_distance = msg.range

    def cb_landing_status(self,msg):
        print("Received landing Activity while hovering: ", msg.data)
        if msg.data == "LANDING":
            self.flying_status = msg.data

    def hover(self):
        if (self.flying_status == "Drone_lifted"):
            while (rospy.is_shutdown() is False) and (self.flying_status != "LANDING"):

                self.custom_activity_pub.publish("HOVER")

                if (self.hover_hight - 0.03 <= self.local_pose.pose.position.z <= self.hover_hight + 0.03):
                    print("AdvanDiptera is hovering")
                else:
                    print("correcting hover altitude")
                    Basic_movement().move_in_z(self.hover_hight - self.local_pose.pose.position.z)

    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        self.takeoff_state = self.modechnge_takeoff()

        while (self.takeoff_height - self.threshold) < self.local_pose.pose.position.z < (self.takeoff_height + self.threshold) and (self.takeoff_height - self.threshold) < self.down_sensor_distance < (self.takeoff_height + self.threshold):
            self.autotakeoff()
            time.sleep(0.2)


        self.offboard_state = self.modechnge()
        time.sleep(0.1)
        Basic_movement().move_to_z(self.takeoff_height)
        time.sleep(0.2)
        
        self.hover()       


if __name__ == '__main__':
    takeoff = Lift_Off()
    takeoff.start()
    time.sleep(5) 




