#!/usr/bin/env python
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, BatteryStatus
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist ,Pose
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
from quaternion import Quaternion
import time
import math
import threading
import yaml

from random import seed
from random import randint



# Path of the yaml file

class Arming_Modechng:

    def __init__(self):
       
        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/data.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():
                if key == "imu":
                    self.imu = value
                if key == "gps":
                    self.gps = value
                if key == "local_pose":
                    self.local_pose = value
                if key == "current_heading":
                    self.current_heading = value
                if key == "local_enu_position":
                    self.local_enu_position = value
                if key == "cur_target_pose":
                    self.cur_target_pose = value
                if key == "global_target":
                    self.global_target = value
                if key == "arm_state":
                    self.arm_state = value
                if key == "offboard_state":
                    self.offboard_state = value
                if key == "frame":
                    self.frame = value
        
        self.battery_voltage = None
        self.battery_current = None 
        self.battery_voltage_remaining = None
    
        self.firstpose = True
        
        '''
        ros subscribers
        '''
        # All the mavros topics can be found here: http://wiki.ros.org/mavros
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback) # Subscriber of the local position by mavros 
                                                                                                                     # http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback) # Subscriber of the State by mavros http://docs.ros.org/api/mavros_msgs/html/msg/State.html
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback) # Subscriber of the global position by mavros 
                                                                                                        # http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback) # Subscriber of imu data by mavros http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
       # self.local_battery_sub = rospy.Subscriber("/mavros/battery", BatteryStatus, self.cb_local_battery)

        '''
        ros publishers
        '''
        # It publishes to the position we want to move http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        '''
        ros services
        '''
        
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) # Service of Arming status http://docs.ros.org/api/mavros_msgs/html/srv/CommandBool.html
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode) # Service mode provided by mavros http://docs.ros.org/api/mavros_msgs/html/srv/SetMode.html
        rospy.init_node("offboard_node") # Initialize the node                                                                                  # Custom modes can be seing here: http://wiki.ros.org/mavros/CustomModes


        print("Px4 Controller Initialized!")



    # Callback function for the battery
    def cb_local_battery(self, msg):
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current 
        self.battery_voltage_remaining = msg.remaining

    # Prints the status of the battery
    def print_battery_status(self):
        stringtorospy_1 = ("Current voltage: " + str(self.battery_voltage))
        stringtorospy_2 = ("Current current: " + str(self.battery_current))
        stringtorospy_3 = ("Remaining: " + str(self.battery_voltage_remaining))

        if self.battery_voltage < 13:
            rospy.logfatal(stringtorospy_1)
            rospy.logfatal(stringtorospy_2)
            rospy.logfatal(stringtorospy_3)

        elif self.battery_voltage < 14:
            rospy.logwarn(stringtorospy_1)
            rospy.logwarn(stringtorospy_2)
            rospy.logwarn(stringtorospy_3)

        else:
            rospy.loginfo(stringtorospy_1)
            rospy.loginfo(stringtorospy_2)
            rospy.loginfo(stringtorospy_3)


    # Function to create the message PositionTarget
    def construct_target(self, x, y, z, yaw, yaw_rate = 1): 
        target_raw_pose = PositionTarget() # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 9 

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose



    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''

    # Function that sees if our position and the target one is between our threshold 
    def position_distance(self, cur_p, target_p, threshold=0.1): 
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x) # Calculates the absolute error between our position and the target
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold): # the threshold is between the sum of our three values
            return True
        else:
            return False

    # Callbacks called in the initialization of the subscription to the mavros topics

    def local_pose_callback(self, msg): 
        self.local_pose = msg
        self.local_enu_position = msg


    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode


    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw



    def gps_callback(self, msg):
        self.gps = msg


    def FLU2ENU(self, msg):

        # Converts the position of the map into the perspective of the drone using the following equations
        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading) # x*cos(alpha) - y*sin(alpha)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading) # x*sin(alpha) + y*cos(alpha)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z

    # Callback of the target position
    def set_target_position_callback(self, msg):

        print("Received New Position Task!")

        # Depending of what are we looking we will look for a position respect the drone or respect the map
        if msg.header.frame_id == 'base_link': 
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg) # Calls this function to get the ENU values


            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

            self.cur_target_pose = self.construct_target(ENU_X,
                                                         ENU_Y,
                                                         ENU_Z,
                                                         self.current_heading)


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            print("local ENU frame")

            self.cur_target_pose = self.construct_target(msg.pose.position.x,
                                                         msg.pose.position.y,
                                                         msg.pose.position.z,
                                                         self.current_heading)






# --------------------------------------------------------------------------------------------------------------


    def set_target_yaw_callback(self, msg): 
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0 # Converts the data into degrees from radians (pi = 180 degrees)
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     yaw_deg)


    # It returns the degrees we need to move, our desired rotation
    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad 

    #Arms the drone
    def arm(self): 
        if self.armService(True):
            rospy.loginfo("Drone armed!")
            return True
        else:
            print("Vehicle arming failed!")
            rospy.logerr("Drone arming failed!")
            return False

    #Disarms the drone
    def disarm(self): 

        if self.armService(False):
            rospy.loginfo("Drone disarmed!")
            return True
        else:
            print("Vehicle disarming failed!")
            rospy.logerr("Drone disarming failed!")
            return False


    # Hover the drone in the actual position
    def hover(self): 
        for i in range(800):
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)
            self.local_target_pub.publish(self.cur_target_pose)
            time.sleep(0.005)

    def take_off(self): 
        for i in range(800):
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z + 1, self.current_heading)
            self.local_target_pub.publish(self.cur_target_pose)
            time.sleep(0.005)

    def land(self): 
        for i in range(800):
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z - 1, self.current_heading)
            self.local_target_pub.publish(self.cur_target_pose)
            time.sleep(0.005)

    def offboard(self): 
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vehicle Offboard failed")
            rospy.loginfo("Vehicle Offboard failed!")
            return False

    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)

        time.sleep(2)
        
        for i in range(20):
            
            self.arm_state = self.arm()    # arms the drone
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)
            self.local_target_pub.publish(self.cur_target_pose)
            self.offboard_state = self.offboard()
            time.sleep(0.1)
            


#########################################################################################################################

if __name__ == '__main__':

    try:
        print("Waiting for Advandiptera brain")
        arm = Arming_Modechng()

        # Arming + liftoff + hover
        arm.start()
        #arm.print_battery_status()
        #if arm.battery_voltage > 13:

        arm.take_off()

        arm.hover()

        arm.land()
  
        #arm.print_battery_status()

        #else:
        #   rospy.logerr("Low battery level, disarming")

    except rospy.ROSInterruptException: pass
    
    
