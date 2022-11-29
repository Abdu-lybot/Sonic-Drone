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
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, Twist
import sys

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
                if key == "initial_thrust":
                    self.init_thrust = value

                if key == "Hover_sensor_altitude":
                    self.hover_sensor_altitude = value
                if key == "Hover_sensor_altitude_max":
                    self.hover_sensor_altitude_max = value
                if key == "Hover_sensor_altitude_min":
                    self.hover_sensor_altitude_min = value
                if key == "Landing_sensor_altitude_min":
                    self.landing_sensor_altitude_min = value


                if key == "Liftoff_thrust":
                    self.Liftoff_thrust = value
                if key == "Hover_thrust":
                    self.hover_thrust = value
		if key == "Landing_thrust":
                    self.Landing_thrust = value
                if key == "accumulating_thrust":
                    self.accumulating_thrust = value
                if key == "Deaccumulating_thrust":
                    self.Deaccumulating_thrust = value
	

		if key == "Time_between_messages":
                    self.Time_between_messages = value

                if key == "Liftoff_time":
                    self.Liftoff_time = value
                if key == "Hover_time":
                    self.hover_time = value
		if key == "Max_time_landing":
                    self.Max_time_landing = value
		if key == "Secure_time_landing":
                    self.Secure_time_landing = value

        self.down_sensor_distance = None 
        self.printing_value = 0 
        self.current_heading = None
        rospy.init_node("Arming_safety_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.attitude_target_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)


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

    def cb_down_sensor(self, msg):
        self.down_sensor_distance = msg.range

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
        
#----------------------arming services----------------------------
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
        
#----------------------messages constructor----------------------------
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
    
    '''    #we should use this in our functions rather then rewriting the message
    def construct_target_attitude(self, body_x = 0, body_y = 0, body_z = 0, thrust = 0.3):
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
    '''

    #----------------------recursive functions----------------------------


    def lift_off_rec(self, thrust ,beh_type ,time_flying):
        if (time_flying <= 0 or self.down_sensor_distance <= self.hover_sensor_altitude_min) and beh_type == "TAKE OFF": 
            print ("time of lift off has ended")
            beh_type = "HOVER"
            return self.hover_rec(thrust ,beh_type, self.Hover_time)


        elif beh_type == "TAKE OFF" and thrust >= self.Liftoff_thrust and self.down_sensor_distance <= self.hover_sensor_altitude_min:
            print("The drone is lifting off at constant thrust")
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            thrust = self.Liftoff_thrust
            target_raw_attitude.thrust = thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time.sleep(self.Time_between_messages) #was 0.005   (now 50hz ,500loops)
            time_flying = time_flying - self.Time_between_messages
            return self.lift_off_rec(target_raw_attitude.thrust, beh_type, time_flying)

        elif beh_type == "TAKE OFF":
            print("Lifting the drone up slowly")
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            target_raw_attitude.thrust = thrust + self.accumulating_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time_flying = time_flying - self.Time_between_messages
            time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
            return self.lift_off_rec(target_raw_attitude.thrust, beh_type, time_flying)


    def hover_rec(self, thrust , beh_type, time_flying): 

        if self.beh_type == 'HOVER' and time_flying <= 0:
            print("Hovering time ended")
            return self.landing_rec(target_raw_attitude.thrust, beh_type, self.Max_time_landing)            

        elif beh_type == 'HOVER' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
            print("The drone is hovering")
            beh_type = 'HOVER'
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            thrust = self.hover_thrust
            target_raw_attitude.thrust = thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time.sleep(self.Time_between_messages) #was 0.005   (now 50hz ,500loops)
            time_flying = time_flying - self.Time_between_messages
            return self.hover_rec(target_raw_attitude.thrust, beh_type, time_flying)
				
        elif beh_type == 'HOVER' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
            print("Recovering hover position - going down")
            beh_type = 'HOVER'
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            thrust = self.hover_thrust
            target_raw_attitude.thrust = thrust - self.Deaccumulating_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
            time_flying = time_flying - self.Time_between_messages
            return self.hover_rec(target_raw_attitude.thrust, beh_type, time_flying)
			
        elif beh_type == 'HOVER' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
            print("Recovering hover position - going up")
            beh_type = 'HOVER'
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            thrust = self.hover_thrust
            target_raw_attitude.thrust = thrust + self.Deaccumulating_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
            time_flying = time_flying - self.Time_between_messages
            return self.hover_rec(target_raw_attitude.thrust, beh_type, time_flying)


    def landing_rec(self, thrust, beh_type, time_flying):
        if (time_flying <= 0 or self.down_sensor_distance <= self.landing_sensor_altitude_min) and beh_type == "LANDING": 
            print("Entering secure landing")
            return self.secure_landing_phase_rec(thrust, beh_type, self.Secure_time_landing)

        elif thrust > self.Landing_thrust: #and beh_type == "HOVER":
            print("Landing the drone down slowly")
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            target_raw_attitude.thrust = thrust
            thrust = thrust - self.accumulating_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time_flying = time_flying - self.Time_between_messages 
            time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
            return self.landing_rec(thrust, beh_type, time_flying)   #bublishing a constant parameter "not
        elif thrust <= self.Landing_thrust: #and beh_type == "HOVER":
            print("Landing the drone down slowly")
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            target_raw_attitude.thrust = self.Landing_thrust
            thrust = self.Landing_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time_flying = time_flying - self.Time_between_messages 
            time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
            return self.landing_rec(thrust, beh_type, time_flying)   #bublishing a constant parameter "not updating thrust argument"

    def secure_landing_phase_rec(self, thrust, time_flying):
        if time_flying <= 0:
            return True
        else:
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            target_raw_attitude.thrust = self.Landing_thrust
            thrust = self.Landing_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time_flying = time_flying - self.Time_between_messages 
            time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
            return self.secure_landing_phase_rec(thrust, beh_type, time_flying)   #bublishing a constant parameter "not updating thrust argument"

#----------------------change modes----------------------------

    def modechnge(self):
        #rospy.init_node("offboard_node")
        if self.flightModeService(custom_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False        
  
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
        #self.cur_target_attitude = self.construct_target_attitude()

        #why are we doing this loop
        for i in range(20):
            
            self.arm_state = self.arm()    # arms the drone
            #self.attitude_target_pub.publish(self.cur_target_attitude)
            self.offboard_state = self.modechnge()
            time.sleep(0.1)
        self.change_python_recursive_limit()
          
    def change_python_recursive_limit(self):
        timers =  [self.Liftoff_time, self.hover_time, self.Max_time_landing, self.Secure_time_landing]
        timers.sort()
        recursions = timers[3] / self.Time_between_messages
        sys.setrecursionlimit(recursions) 
            
#####################################################################################################            
if __name__ == '__main__':

    try:
        Gpio_start().start()
        time.sleep(3.5)
        print("Waiting for Advandiptera brain")
        arm = Arming_Modechng()
        arm.start()
        arm.lift_off_rec(arm.init_thrust, "Lift_off", arm.Liftoff_time)
    except rospy.ROSInterruptException: pass
