#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from quaternion import Quaternion
import time, math
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, Twist


class Lidar_Avoidance():

    def __init__(self):
        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():

                if key == "Avoiding_obstacle_distance_min":
                    self.Avoiding_obstacle_distance_min = value
                if key == "Blocking_movement_distance_min":
                    self.Blocking_movement_distance_min = value
                if key == "Unblocking_movement_distance_min":
                    self.Unblocking_movement_distance_min = value

                # Time between messages
		if key == "Time_between_messages":
                    self.Time_between_messages = value


		if key == "Avoiding_obstacle_time":
                    self.Avoiding_obstacle_time = value


        self.skip_to_land = False   # In case of an error it will skip al the movements and will go automatically to landing 

        # States - UNBLOCK/BLOCK/AVOID
        self.stateRight = "UNBLOCK" 
        self.stateLeft = "UNBLOCK"
        self.stateForward = "UNBLOCK"
        self.stateBack = "UNBLOCK"

        # State Avoiding obstacle
        self.stateRightAvoiding = False 
        self.stateLeftAvoiding = False
        self.stateForwardAvoiding = False
        self.stateBackAvoiding = False
        self.stateAvoiding = False

        self.front_sensor_distance = None
        self.back_sensor_distance = None
        self.right_sensor_distance = None
        self.left_sensor_distance = None  
        self.down_sensor_distance = None
        self.down_sensor_distance_old = None 
        self.down_sensor_changed = False
        self.down_sensor_distance_higher = False

	self.beh_type = None
       
        # variable to know if the drone is being controlled by the keyboard
        self.keyboardcontrolstate = None

        self.printing_value = 0 
        self.current_heading = None

        rospy.init_node("Lidar_Obstacle_Avoidance_node")

        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.attitude_target_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)

        # Custom subscribers
        self.front_obstacle_detection_sub = rospy.Subscriber("/Front_Obstacle", Range, self.front_obstacle_detection_callback)
        self.back_obstacle_detection_sub = rospy.Subscriber("/Back_Obstacle", Range, self.back_obstacle_detection_callback)
        self.left_obstacle_detection_sub = rospy.Subscriber("/Left_Obstacle", Range, self.left_obstacle_detection_callback)
        self.right_obstacle_detection_sub = rospy.Subscriber("/Right_Obstacle", Range, self.right_obstacle_detection_callback)
        self.lidar_avoidance_pub = rospy.Publisher('/custom/lidaravoidance', String, queue_size=10) 
  
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

    def imu_callback(self, msg):
        global global_imu
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True
        
 
    def euler2quaternion(self, roll = 0, pitch = 0, yaw = 0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
        
    def calculate_recursions(self, total_time):
	recursions = total_time/self.Time_between_messages
        return int(recursions)

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))
        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        return [w, x, y, z]
		


################################# AVOIDING OBSTACLE ########################################
# This functions detect the obstacle and sends the order to move the drone in the opposite direction if needed
# There are three ranges, depending what the lidar sense, it will enter to one zone or another

    def front_obstacle_detection_callback(self, msg):
        self.front_sensor_distance = msg.range
        sensor_distance = msg.range
 
        if sensor_distance <= self.Avoiding_obstacle_distance_min and self.stateForward != "AVOID":
            self.stateForward = "AVOID"  

        elif sensor_distance > self.Avoiding_obstacle_distance_min and self.stateForward != "UNBLOCK":
            self.stateForward = "UNBLOCK"
           
    def back_obstacle_detection_callback(self, msg):
        self.back_sensor_distance = msg.range
        sensor_distance = msg.range
 
        if sensor_distance <= self.Avoiding_obstacle_distance_min and self.stateBack != "AVOID":
            self.stateBack = "AVOID"

        elif sensor_distance > self.Avoiding_obstacle_distance_min and self.stateBack != "UNBLOCK":
            self.stateBack = "UNBLOCK"

    def right_obstacle_detection_callback(self, msg):
        self.right_sensor_distance = msg.range
        sensor_distance = msg.range
 
        if sensor_distance <= self.Avoiding_obstacle_distance_min and self.stateRight != "AVOID":
            self.stateRight = "AVOID"

        elif sensor_distance > self.Avoiding_obstacle_distance_min and self.stateRight != "UNBLOCK":
            self.stateRight = "UNBLOCK"

    def left_obstacle_detection_callback(self, msg):
        self.left_sensor_distance = msg.range
        sensor_distance = msg.range
 
        if sensor_distance <= self.Avoiding_obstacle_distance_min and self.stateLeft != "AVOID":
            self.stateLeft = "AVOID"

        elif sensor_distance > self.Avoiding_obstacle_distance_min and self.stateLeft != "UNBLOCK":
            self.stateLeft = "UNBLOCK"



 #----------------------start function---------------------------- 

    def start(self):
        for i in range(1000000): # Waits 5 seconds for initialization
            if self.stateLeft == "UNBLOCK" and self.stateRight == "UNBLOCK" and self.stateBack == "UNBLOCK" and self.stateForward == "UNBLOCK":
                self.lidar_avoidance_pub.publish(String("No Obstacle"))

            # Lineal Directions
            elif self.stateForward == "AVOID":  # Obstacle Front
                if self.stateLeft == "AVOID":
                    self.lidar_avoidance_pub.publish(String("Front Left"))
                elif self.stateRight == "AVOID":
                    self.lidar_avoidance_pub.publish(String("Front Right"))
                else:
                    self.lidar_avoidance_pub.publish(String("Front"))
            elif self.stateBack == "AVOID":  # Obstacle Back
                if self.stateLeft == "AVOID":
                    self.lidar_avoidance_pub.publish(String("Back Left"))
                elif self.stateRight == "AVOID":
                    self.lidar_avoidance_pub.publish(String("Back Right"))
                else:
                    self.lidar_avoidance_pub.publish(String("Back"))

            elif self.stateLeft == "AVOID":  # Obstacle Left
                self.lidar_avoidance_pub.publish(String("Left"))

            elif self.stateRight == "AVOID":  # Obstacle Right
                self.lidar_avoidance_pub.publish(String("Right"))

            time.sleep(self.Avoiding_obstacle_time)

    def start2(self):
        for i in range(1000000): # Waits 5 seconds for initialization
            if self.stateLeft == "UNBLOCK" and self.stateRight == "UNBLOCK" and self.stateBack == "UNBLOCK" and self.stateForward == "UNBLOCK":
                self.lidar_avoidance_pub.publish(String("No Obstacle"))

            # Lineal Directions
            elif self.stateForward == "AVOID":  # Obstacle Front
                self.lidar_avoidance_pub.publish(String("Front"))

            elif self.stateBack == "AVOID":  # Obstacle Back
                self.lidar_avoidance_pub.publish(String("Back"))

            elif self.stateLeft == "AVOID":  # Obstacle Left
                self.lidar_avoidance_pub.publish(String("Left"))

            elif self.stateRight == "AVOID":  # Obstacle Right
                self.lidar_avoidance_pub.publish(String("Right"))

            time.sleep(self.Avoiding_obstacle_time) 

 
#####################################################################################################            
if __name__ == '__main__':

    ldr = Lidar_Avoidance()
    ldr.start()

