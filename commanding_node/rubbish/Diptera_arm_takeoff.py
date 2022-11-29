#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from sensor_msgs.msg import Imu, NavSatFix
from quaternion import Quaternion
import time
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop


class Arming_Modechng():

    def __init__(self):
        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():
                if key == "initial_x_pos":
                    self.init_x = value
                if key == "initial_y_pos":
                    self.init_y = value
                if key == "initial_z_pos":
                    self.init_z = value
        self.status_thrust = None
        self.current_heading = None
        rospy.init_node("Arming_safety_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.attitude_target_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def imu_callback(self, msg):
        global global_imu
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True

    def arm(self):
        if self.armService(True):
            rospy.loginfo("AdvanDiptera is Armed")
            return True
        else:
            rospy.loginfo("Failed to arm AdvanDiptera")
            return False

    def disarm(self):
        if self.armService(False):
            rospy.loginfo("Advandiptera succesfully disarmed")
            return True
        else:
            rospy.loginfo("Failed to disarm AdvanDiptera")
            return False

    def construct_target(self, x, y, z, yaw, yaw_rate=1):
        target_raw_pose = PositionTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
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
        
    def construct_target_attitude(self, body_x = 0, body_y = 0, body_z = 0, thrust = 0.5):
        target_raw_attitude = AttitudeTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_attitude.header.stamp = rospy.Time.now()
        #target_raw_attitude.orientation. = self.imu.orientation
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE \
                                    + AttitudeTarget.IGNORE_ATTITUDE

        target_raw_attitude.body_rate.x = body_x # ROLL_RATE
        target_raw_attitude.body_rate.y = body_y # PITCH_RATE
        target_raw_attitude.body_rate.z = body_z # YAW_RATE
        target_raw_attitude.thrust = thrust
        return target_raw_attitude
 ##############################################################
    def body_recursion(self, thrust):
        if self.status_thrust == "up":
            if (thrust <= -1):
                return True
                
            else:
                target_raw_attitude = AttitudeTarget() 
                target_raw_attitude.header.stamp = rospy.Time.now()
                target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ATTITUDE
                target_raw_attitude.thrust = 0.5 
                
                target_raw_attitude.body_rate.x = 0 # ROLL_RATE
                target_raw_attitude.body_rate.y = thrust-0.003 # PITCH_RATE
                target_raw_attitude.body_rate.z = 0 # YAW_RATE
                
                self.attitude_target_pub.publish(target_raw_attitude)
                time.sleep(0.1)
                return self.body_recursion(target_raw_attitude.body_rate.y)
        
    
    
    
    ############################################################################
    def cb_local_pose(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg
    
    def autotakeoff2(self):
        self.cur_target_attitude = self.construct_target_attitude()
        self.attitude_target_pub.publish(self.cur_target_attitude) 

    def autotakeoff(self):
        if (self.takeoffService(min_pitch = 2, latitude = 0, longitude = 0, altitude = 1.2, yaw = self.current_heading)):
            rospy.loginfo("AdvanDiptera is Hovering")
            self.flying_status = "Drone_lifted"
            return True 
        else:
            rospy.loginfo("Failed to takeoff AdvanDiptera")
            return False
    
    
    def thrust_recursion(self, thrust):
        if self.status_thrust == "up":
            if (thrust >= 1):
                print ("the thrust has reached its desired point")
                print ("the motors should start slowing down...")
                target_raw_attitude = AttitudeTarget() 
                target_raw_attitude.header.stamp = rospy.Time.now()
                target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE \
                                        + AttitudeTarget.IGNORE_ATTITUDE
                target_raw_attitude.thrust = 1
                self.attitude_target_pub.publish(target_raw_attitude)
                time.sleep(0.1)
                self.status_thrust = "slow down"
                return self.thrust_recursion(target_raw_attitude.thrust)
                
            else:
                target_raw_attitude = AttitudeTarget() 
                target_raw_attitude.header.stamp = rospy.Time.now()
                target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE \
                                        + AttitudeTarget.IGNORE_ATTITUDE
                target_raw_attitude.thrust = thrust + 0.003
  
                self.attitude_target_pub.publish(target_raw_attitude)
                time.sleep(0.1)
                return self.thrust_recursion(target_raw_attitude.thrust)
        elif self.status_thrust == "slow down":
            if (thrust <= 0):
                print ("the motors should stop")
                self.status_thrust = "stop"
                return True
                
            else:
                target_raw_attitude = AttitudeTarget() 
                target_raw_attitude.header.stamp = rospy.Time.now()
                target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE \
                                        + AttitudeTarget.IGNORE_ATTITUDE
                target_raw_attitude.thrust = thrust - 0.003
  
                self.attitude_target_pub.publish(target_raw_attitude)
                time.sleep(0.1)
                return self.thrust_recursion(target_raw_attitude.thrust)
        
        
        
        
# ----------------------change modes----------------------------
    def modechnge_takeoff(self):
        if self.flightModeService(custom_mode='AUTO.TAKEOFF'):       # http://wiki.ros.org/mavros/CustomModes
            rospy.loginfo("succesfully changed mode to AUTO.TAKEOFF")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False

    def modechnge(self):
        #rospy.init_node("offboard_node")
        if self.flightModeService(custom_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False        
  
############################################################ START##################################################        
    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        self.cur_target_pose = self.construct_target(self.init_x, self.init_y, self.init_z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)
        
        self.arm_state = self.arm()
        self.offboard_state = self.modechnge()
        time.sleep(2)
        self.cur_target_attitude = self.construct_target_attitude()
        for i in range(20):
            
            self.arm_state = self.arm()    # arms the drone
            self.attitude_target_pub.publish(self.cur_target_attitude)
            self.offboard_state = self.modechnge()
            time.sleep(0.1)

        for i in range(150):
            #self.local_target_pub.publish(self.cur_target_pose) # Publish the drone position we initialite during the first 2 seconds
            #self.arm_state = self.arm()    # arms the drone
            #self.offboard_state = self.modechnge() # Calls the function offboard the will select the mode Offboard
            self.attitude_target_pub.publish(self.cur_target_attitude)
            time.sleep(0.1)
        self.status_thrust = "up"
        self.body_recursion(0.003)    
            
            
            
            
if __name__ == '__main__':

    try:
        Gpio_start().start()
        time.sleep(3.5)
        print("Waiting for Advandiptera brain")
        arm = Arming_Modechng()
        arm.start()

    except rospy.ROSInterruptException: pass
