#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from sensor_msgs.msg import Imu, NavSatFix, Range
from quaternion import Quaternion
import time
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, Twist


class Arming_Modechng():

    def __init__(self):
        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():
                # Position
                if key == "initial_x_pos":
                    self.init_x = value
                if key == "initial_y_pos":
                    self.init_y = value
                if key == "initial_z_pos":
                    self.init_z = value
                if key == "initial_thrust":
                    self.init_thrust = value

                # Distances
                if key == "Hover_sensor_altitude":
                    self.hover_sensor_altitude = value
                if key == "Hover_sensor_altitude_max":
                    self.hover_sensor_altitude_max = value
                if key == "Hover_sensor_altitude_min":
                    self.hover_sensor_altitude_min = value
                if key == "Landing_sensor_altitude_min":
                    self.landing_sensor_altitude_min = value
                if key == "Avoiding_obstacle_distance_min":
                    self.Avoiding_obstacle_distance_min = value
                if key == "Blocking_movement_distance_min":
                    self.Blocking_movement_distance_min = value
                if key == "Unblocking_movemente_distance_min":
                    self.Unblocking_movemente_distance_min = value

                # Thrusts
                if key == "Liftoff_thrust":
                    self.Liftoff_thrust = value
                if key == "Hover_thrust":
                    self.hover_thrust = value
                if key == "Moving_thrust":
                    self.Moving_thrust = value
                if key == "Landing_thrust":
                    self.Landing_thrust = value
                if key == "accumulating_thrust":
                    self.accumulating_thrust = value
                if key == "Deaccumulating_thrust":
                    self.Deaccumulating_thrust = value
	
                # Time between messages
                if key == "Time_between_messages":
                    self.Time_between_messages = value

                # Movement Timers
                if key == "Liftoff_time":
                    self.Liftoff_time = value
                if key == "Hover_time":
                    self.hover_time = value
                if key == "Moving_forward_time":
                    self.Moving_forward_time = value
                if key == "Secure_time_landing":
                    self.Secure_time_landing = value	
                if key == "Max_time_landing":
                    self.Max_time_landing = value	
                if key == "Avoiding_obstacle_time":
                    self.Avoiding_obstacle_time = value					
		 
                # Angles
                if key == "angle_roll_left":
                    self.angle_roll_left = value
                if key == "angle_roll_right":
                    self.angle_roll_right = value
                if key == "angle_pitch_back":
                    self.angle_pitch_back = value	
                if key == "angle_pitch_forward":
                    self.angle_pitch_forward = value


			
        self.down_sensor_distance = None 
        self.front_sensor_distance = None
        self.back_sensor_distance = None
        self.right_sensor_distance = None
        self.left_sensor_distance = None      
        
        self.printing_value = 0 
        self.current_heading = None
        
        
        
        # Initialize the variables to False
        self.blockingMovementRight = False   
        self.blockingMovementLeft = False 
        self.blockingMovementBack = False
        self.blockingMovementFront = False
        
        rospy.init_node("Arming_safety_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.attitude_target_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)
        
        # Custom subscribers
        self.front_obstacle_detection_sub = rospy.Subscriber("/Front_Obstacle", Range, self.front_obstacle_detection_callback)
        self.back_obstacle_detection_sub = rospy.Subscriber("/Back_Obstacle", Range, self.back_obstacle_detection_callback)
        self.left_obstacle_detection_sub = rospy.Subscriber("/Left_Obstacle", Range, self.left_obstacle_detection_callback)
        self.right_obstacle_detection_sub = rospy.Subscriber("/Right_Obstacle", Range, self.right_obstacle_detection_callback)


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

    def front_obstacle_detection_callback(self, msg):
        self.front_sensor_distance = msg.range
        if self.front_sensor_distance <= self.Avoiding_obstacle_distance_min:
            self.blockingMovementFront = True   
            self.moving_backward_rec(self.Avoiding_obstacle_time)
        elif self.front_sensor_distance <= self.Blocking_movement_distance_min:
            self.blockingMovementFront = True
        elif self.front_sensor_distance >= self.Unblocking_movement_distance_min:
            self.blockingMovementFront = False
           
    def back_obstacle_detection_callback(self, msg):
        self.back_sensor_distance = msg.range
        if self.back_sensor_distance <= self.Avoiding_obstacle_distance_min:
            self.blockingMovementBack = True   
            self.moving_forward_rec(self.Avoiding_obstacle_time)
        elif self.back_sensor_distance <= self.Blocking_movement_distance_min:
            self.blockingMovementBack = True
        elif self.back_sensor_distance >= self.Unblocking_movement_distance_min:
            self.blockingMovementBack = False

    def right_obstacle_detection_callback(self, msg):
        self.right_sensor_distance = msg.range
        if self.right_sensor_distance <= self.Avoiding_obstacle_distance_min:
            self.blockingMovementRight = True   
            self.moving_left_rec(self.Avoiding_obstacle_time)
        elif self.right_sensor_distance <= self.Blocking_movement_distance_min:
            self.blockingMovementRight = True
        elif self.right_sensor_distance >= self.Unblocking_movement_distance_min:
            self.blockingMovementRight = False

    def left_obstacle_detection_callback(self, msg):
        self.left_sensor_distance = msg.range
        if self.left_sensor_distance <= self.Avoiding_obstacle_distance_min:
            self.blockingMovementLeft = True   
            self.moving_right_rec(self.Avoiding_obstacle_time)
        elif self.left_sensor_distance <= self.Blocking_movement_distance_min:
            self.blockingMovementLeft = True
        elif self.left_sensor_distance >= self.Unblocking_movement_distance_min:
            self.blockingMovementLeft = False
            

    def euler2quaternion(self, roll = 0, pitch = 0, yaw = 0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
        
    def calculate_recursions(self, total_time):
        recursions = total_time/self.Time_between_messages
        return int(recursions)
		
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
    

    def construct_target_attitude(self, body_x = 0, body_y = 0, body_z = 0, thrust=0):
        target_raw_attitude = AttitudeTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.orientation = self.imu.orientation

        target_raw_attitude.body_rate.x = body_x # ROLL_RATE
        target_raw_attitude.body_rate.y = body_y # PITCH_RATE
        target_raw_attitude.body_rate.z = body_z # YAW_RATE
        target_raw_attitude.thrust = thrust
        self.attitude_target_pub.publish(target_raw_attitude)



    #----------------------recursive functions----------------------------
	
    def lift_off_rec(self, thrust, time_flying):
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = "TAKE OFF"
        for i in range(recursions):
            print (i)
            if self.beh_type == "TAKE OFF" and thrust < self.Liftoff_thrust: #and beh_type == "HOVER":
                print("Lifting the drone up slowly")
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust + self.accumulating_thrust
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)

            elif self.beh_type == "TAKE OFF" and thrust >= self.Liftoff_thrust and self.down_sensor_distance <= self.hover_sensor_altitude_min:
                print("The drone is lifting off at constant thrust")
                thrust = self.Liftoff_thrust
                self.construct_target_attitude(0,0,0,thrust)
                time.sleep(self.Time_between_messages) #was 0.005   (now 50hz ,500loops)
            else:
                break
        self.beh_type = 'HOVER'
        print ("time of lifting off has ended")
        self.hover_rec(self.hover_time)


    def hover_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        for i in range(recursions):
            print (i)
            print(self.down_sensor_distance)        
            if self.beh_type == 'HOVER' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                print("The drone is hovering")
                self.beh_type = 'HOVER'

                thrust = self.hover_thrust
                self.construct_target_attitude(0,0,0,thrust)
                time.sleep(self.Time_between_messages) #was 0.005   (now 50hz ,500loops)
				
            elif self.beh_type == 'HOVER' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                print("Recovering hover position - going down")
                self.beh_type = 'HOVER'

                thrust = self.hover_thrust
                self.construct_target_attitude(0,0,0,thrust - self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
				
            elif self.beh_type == 'HOVER' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                print("Recovering hover position - going up")
                self.beh_type = 'HOVER'

                thrust = self.hover_thrust
                self.construct_target_attitude(0,0,0,thrust + self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)

				
        print ("time of hovering has ended")
        #self.beh_type = "LANDING"
        #self.landing_rec()
        self.moving_forward_rec(self.Moving_forward_time)

    def moving_right_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = 'MOVING'
        for i in range(recursions):
            print (i)
            print(self.down_sensor_distance)        
            if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                print("The drone is moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(angle_roll_right,0,0,thrust)
                time.sleep(self.Time_between_messages) #was 0.005   (now 50hz ,500loops)
				
            elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                print("Recovering hover position - going down + moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(angle_roll_right,0,0,thrust - self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
				
            elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                print("Recovering hover position - going up + moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(angle_roll_right,0,0,thrust + self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
                
    def moving_left_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = 'MOVING'
        for i in range(recursions):
            print (i)
            print(self.down_sensor_distance)        
            if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                print("The drone is moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(angle_roll_left,0,0,thrust)
                time.sleep(self.Time_between_messages) #was 0.005   (now 50hz ,500loops)
				
            elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                print("Recovering hover position - going down + moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(angle_roll_left,0,0,thrust - self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
				
            elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                print("Recovering hover position - going up + moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(angle_roll_left,0,0,thrust + self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)

    def moving_backward_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = 'MOVING'
        for i in range(recursions):
            print (i)
            print(self.down_sensor_distance)        
            if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                print("The drone is moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(0,self.angle_pitch_back,0,thrust)
                time.sleep(self.Time_between_messages) #was 0.005   (now 50hz ,500loops)
				
            elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                print("Recovering hover position - going down + moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(0,self.angle_pitch_back,0,thrust - self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
				
            elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                print("Recovering hover position - going up + moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(0,self.angle_pitch_back,0,thrust + self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
 




    def moving_forward_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = 'MOVING'
        for i in range(recursions):
            print (i)
            print(self.down_sensor_distance)        
            if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                print("The drone is moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(0,self.angle_pitch_forward,0,thrust)
                time.sleep(self.Time_between_messages) #was 0.005   (now 50hz ,500loops)
				
            elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                print("Recovering hover position - going down + moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(0,self.angle_pitch_forward,0,thrust - self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
				
            elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                print("Recovering hover position - going up + moving forward")
                self.beh_type = 'MOVING'
                thrust = self.Moving_thrust
                self.construct_target_attitude(0,self.angle_pitch_forward,0,thrust + self.Deaccumulating_thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)

				
        print ("time of hovering has ended")
        self.beh_type = "LANDING"
        self.landing_rec()
	
    def landing_rec(self):                       # Landing phase
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Max_time_landing)
        print(recursions)
        thrust = self.hover_thrust
        for i in range(recursions):
            print self.down_sensor_distance
            if self.down_sensor_distance <= self.landing_sensor_altitude_min:
                break

            elif thrust > self.Landing_thrust: #and beh_type == "HOVER":
                print("Landing the drone down slowly")
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust - self.accumulating_thrust
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)

            elif thrust <= self.Landing_thrust: #and beh_type == "HOVER":
                print("Landing the drone down slowly")

                thrust = self.Landing_thrust
                self.construct_target_attitude(0,0,0,thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)				
        self.secure_landing_phase_rec2()

    def secure_landing_phase_rec2(self):           # Secure landing part - last cm
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Secure_time_landing)	
        thrust = self.hover_thrust
        for i in range(recursions):
            if thrust <= 0:
                break

            if i < 400:
                print("Smooth landing")

                thrust = self.Landing_thrust
                self.construct_target_attitude(0,0,0,thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)

            else: 
                print("Landing")
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust - self.accumulating_thrust
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)	
        self.disarm()

    def secure_landing_phase_rec(self):           # Secure landing part - last cm
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Secure_time_landing)	
        thrust = self.hover_thrust
        for i in range(recursions):
            if thrust <= 0:
                break
            elif i < 200:
                print("Secure hover before landing")

                thrust = self.hover_thrust
                self.construct_target_attitude(0,0,0,thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)

            elif i < 700:
                print("Smooth landing")

                thrust = self.hover_thrust - self.Deaccumulating_thrust
                self.construct_target_attitude(0,0,0,thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)
            else: 
                print("Landing")

                thrust = thrust - self.accumulating_thrust
                self.construct_target_attitude(0,0,0,thrust)
                time.sleep(self.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)	
        self.disarm()



#----------------------change modes----------------------------

    def modechnge(self):
        #rospy.init_node("offboard_node")
        if self.flightModeService(custom_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False        
 #----------------------start function---------------------------- 
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
            
 

 
#####################################################################################################            
if __name__ == '__main__':

    try:
        Gpio_start().start()
        time.sleep(3.5)
        print("Waiting for Advandiptera brain")
        arm = Arming_Modechng()
        arm.start()
        arm.lift_off_rec(arm.init_thrust, arm.Liftoff_time)
    except rospy.ROSInterruptException: pass
