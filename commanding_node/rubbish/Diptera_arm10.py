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

from px_comm.msg import OpticalFlow

class Arming_Modechng():

    def __init__(self):
        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():
                # Initial values
                if key == "initial_x_pos":
                    self.init_x = value
                if key == "initial_y_pos":
                    self.init_y = value
                if key == "initial_z_pos":
                    self.init_z = value
                if key == "initial_thrust":
                    self.init_thrust = value

                # Sensor distances
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
                if key == "Unblocking_movement_distance_min":
                    self.Unblocking_movement_distance_min = value

                # Thrust
                if key == "Liftoff_thrust":
                    self.Liftoff_thrust = value
                    self.Liftoff_thrust_first = self.Liftoff_thrust
                if key == "Hover_thrust":
                    self.hover_thrust = value
		if key == "Landing_thrust":
                    self.Landing_thrust = value
                if key == "Moving_max_thrust":
                    self.Moving_max_thrust = value
		if key == "Moving_min_thrust":
                    self.Moving_min_thrust = value
                if key == "accumulating_thrust_soft":
                    self.accumulating_thrust_soft = value
                if key == "accumulating_thrust":
                    self.accumulating_thrust = value
                if key == "Deaccumulating_thrust":
                    self.Deaccumulating_thrust = value
                if key == "Liftoff_thrust_maximum":
                    self.Liftoff_thrust_maximum = value	

                # Time between messages
		if key == "Time_between_messages":
                    self.Time_between_messages = value

                # Timers
                if key == "Liftoff_time":
                    self.Liftoff_time = value
                if key == "Hover_time":
                    self.hover_time = value
                if key == "Moving_time":
                    self.Moving_time = value
		if key == "Secure_time_landing":
                    self.Secure_time_landing = value	
		if key == "Max_time_landing":
                    self.Max_time_landing = value						
		if key == "Avoiding_obstacle_time":
                    self.Avoiding_obstacle_time = value	
		if key == "Keyboard_control_time":
                    self.Keyboard_control_time = value
		if key == "move_desired_direction_time":
                    self.move_desired_direction_time = value	
		if key == "move_opposite_direction_time":
                    self.move_opposite_direction_time = value 
		if key == "move_hover_time":
                    self.move_hover_time = value
	
                # Angles
                if key == "angle_roll_left":
                    self.angle_roll_left = value
                if key == "angle_roll_right":
                    self.angle_roll_right = value
		if key == "angle_pitch_back":
                    self.angle_pitch_back = value	
		if key == "angle_pitch_forward":
                    self.angle_pitch_forward = value

                if key == "angle_roll_left_hard":
                    self.angle_roll_left_hard = value
                if key == "angle_roll_right_hard":
                    self.angle_roll_right_hard = value
		if key == "angle_pitch_back_hard":
                    self.angle_pitch_back_hard = value	
		if key == "angle_pitch_forward_hard":
                    self.angle_pitch_forward_hard = value

		if key == "angle_pitch_flow_max":
                    self.angle_pitch_flow_max = value
		if key == "angle_roll_flow_max":
                    self.angle_roll_flow_max = value

		if key == "flow_rate_threshold":
                    self.flow_rate_threshold = value

        # FLOW
        self.local_flow_x = 0
        self.local_flow_y = 0
        self.angle_pitch_flow = 0 # positive front/ negative back 
        self.angle_roll_flow = 0 # positive left/ negative right

        self.skip_to_land = False   # In case of an error it will skip al the movements and will go automatically to landing 
	self.Liftoff_thrust_old = None
        self.Landing_thrust_old = None	
        # Initialize the variables to False
        self.blockingMovementRight = False   
        self.blockingMovementLeft = False 
        self.blockingMovementBack = False
        self.blockingMovementFront = False

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

        # Initialize the counters to 0
        self.counterRightAvoidObstacle = 0
        self.counterRightBlockMovement = 0
        self.counterRightUnblockMovement = 0
   
        self.counterLeftAvoidObstacle = 0
        self.counterLeftBlockMovement = 0
        self.counterLeftUnblockMovement = 0

        self.counterForwardAvoidObstacle = 0
        self.counterForwardBlockMovement = 0
        self.counterForwardUnblockMovement = 0

        self.counterBackAvoidObstacle = 0
        self.counterBackBlockMovement = 0
        self.counterBackUnblockMovement = 0

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
        self.lidarcontrolstate = "No Obstacle"

        self.printing_value = 0 
        self.current_heading = None

        rospy.init_node("Arming_safety_node")

        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.attitude_target_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)

        self.local_flow_sub = rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, self.local_flow)

        # Custom subscribers
        self.lidar_obstacle_detection_sub = rospy.Subscriber("/custom/lidaravoidance", String, self.lidar_obstacle_detection_callback)
        self.keyboardcontrol_target_sub = rospy.Subscriber("/custom/keyboardcontrol", String, self.keyboardcontrol_callback)

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
        self.down_sensor_distance_old = self.down_sensor_distance
        self.down_sensor_distance = msg.range
        self.down_sensor_changed = True

        if self.down_sensor_distance_old < self.down_sensor_distance:
            self.down_sensor_distance_higher = True
        else:
            self.down_sensor_distance_higher = False

        if self.down_sensor_distance > 2.5:
            self.skip_to_land = True

    def imu_callback(self, msg):
        global global_imu
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True
        
    def keyboardcontrol_callback(self, msg):
        self.keyboardcontrolstate = msg.data 
        if self.keyboardcontrolstate == "Land":
            self.skip_to_land = True
        if self.keyboardcontrolstate == "Emergency land":
            self.skip_to_land = True
            self.change_thrusts(0.49, 0.49, 0.49)

    def local_flow(self,msg):
        self.local_flow = msg
        self.local_flow_x = msg.flow_x
        self.local_flow_y = msg.flow_y
 
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
		

    def change_thrusts(self, newLiftoff_thrust, newhover_thrust, newLanding_thrust):
        self.Liftoff_thrust = newLiftoff_thrust
        self.hover_thrust = newhover_thrust
        self.Landing_thrust = newLanding_thrust
        self.Moving_max_thrust = self.Liftoff_thrust
        self.Moving_min_thrust = self.Landing_thrust

    def calculate_thrust(self):

        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Liftoff_thrust - self.Landing_thrust
        proportional_thrust = difference_thrust / difference_distance
        thrust = float(self.hover_thrust)

        if self.hover_sensor_altitude_max >= self.down_sensor_distance and self.down_sensor_distance >= self.hover_sensor_altitude_min:
            distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
            thrust = self.Liftoff_thrust - distance_more * proportional_thrust
            print('Thrust: %f    The drone is in the range' % (thrust))           
				
        elif self.hover_sensor_altitude_max <= self.down_sensor_distance: # We have to go down
            thrust = self.Landing_thrust
            print('Thrust: %f    The drone is going down' % (thrust))
				
        elif self.down_sensor_distance <= self.hover_sensor_altitude_min: # We have to go up
            thrust = self.Liftoff_thrust
            print('Thrust: %f    The drone is going up' % (thrust))

        elif self.hover_sensor_altitude_max + 0.5 <= self.down_sensor_distance: # We have to go down + change thrusts variables
            thrust = self.Landing_thrust
            if self.down_sensor_changed == True:
                self.down_sensor_changed = False
                thrust = thrust - 0.005
                self.change_thrusts(thrust + 0.035, thrust + 0.02, thrust)
                thrust = self.Landing_thrust
            print('Thrust: %f    The drone is hovering - going down' % (thrust))
				
        elif self.down_sensor_distance <= self.hover_sensor_altitude_min - 0.05 and thrust <= self.Liftoff_thrust_maximum: # We have to go up + change thrusts variables
            thrust = self.Liftoff_thrust
            if self.down_sensor_changed == True:
                self.down_sensor_changed = False
                thrust = thrust + 0.005
                self.change_thrusts(thrust, thrust - 0.015, thrust - 0.035)
                thrust = self.Liftoff_thrust
            print('Thrust: %f    The drone is hovering - going down' % (thrust))
        else:
            thrust = self.Liftoff_thrust
            print('Thrust: %f    The drone is hovering - maximum thrust obtained' % (thrust))

        return thrust


    def looking_obstacles(self):
        detected = True
        # Hard movements
        if self.lidarcontrolstate == "Front Right Hard":
            self.moving_back_left_rec_hard()
        elif self.lidarcontrolstate == "Front Left Hard":
            self.moving_back_right_rec_hard()
        elif self.lidarcontrolstate == "Back Right Hard":
            self.moving_front_left_rec_hard()
        elif self.lidarcontrolstate == "Back Left Hard":
            self.moving_front_right_rec_hard()

        elif self.lidarcontrolstate == "Right Hard":
            self.moving_left_rec_hard()
        elif self.lidarcontrolstate == "Left Hard":
            self.moving_right_rec_hard()
        elif self.lidarcontrolstate == "Front Hard":
            self.moving_back_rec_hard()
        elif self.lidarcontrolstate == "Back Hard":
            self.moving_forward_rec_hard()

        # Normal movements
        elif self.lidarcontrolstate == "Front Right":
            self.moving_back_left_rec()
        elif self.lidarcontrolstate == "Front Left":
            self.moving_back_right_rec()
        elif self.lidarcontrolstate == "Back Right":
            self.moving_front_left_rec()
        elif self.lidarcontrolstate == "Back Left":
            self.moving_front_right_rec()

        elif self.lidarcontrolstate == "Right":
            self.moving_left_rec()
        elif self.lidarcontrolstate == "Left":
            self.moving_right_rec()
        elif self.lidarcontrolstate == "Front":
            self.moving_back_rec()
        elif self.lidarcontrolstate == "Back":
            self.moving_forward_rec()

        else:
            detected = False

        return detected


    def looking_px4flow(self):
        if self.local_flow_x > self.flow_rate_threshold: # drone moving right, correcting moving left
            self.moving_left_raw_hard_rec(0.15)

        elif self.local_flow_x < - self.flow_rate_threshold: # drone moving left, correcting moving right
            self.moving_right_raw_hard_rec(0.15)

        if self.local_flow_y > self.flow_rate_threshold: # drone moving front, correcting moving back
            self.moving_back_raw_hard_rec(0.15)

        elif self.local_flow_y < - self.flow_rate_threshold: # drone moving back, correcting moving front
            self.moving_forward_raw_hard_rec(0.15)



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
    

    def construct_target_attitude(self, body_x = 0, body_y = 0, body_z = 0, thrust=0, roll_angle = 0, pitch_angle = 0, yaw_angle = 0):

        target_raw_attitude = AttitudeTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_attitude.header.stamp = rospy.Time.now()
        #target_raw_attitude.orientation = self.imu.orientation
        q = self.to_quaternion(roll_angle, pitch_angle, yaw_angle)
        target_raw_attitude.orientation.w = q[0]
        target_raw_attitude.orientation.x = q[1]
        target_raw_attitude.orientation.y = q[2]
        target_raw_attitude.orientation.z = q[3]
        target_raw_attitude.body_rate.x = body_x # ROLL_RATE
        target_raw_attitude.body_rate.y = body_y # PITCH_RATE
        target_raw_attitude.body_rate.z = body_z # YAW_RATE
        target_raw_attitude.thrust = thrust
        self.attitude_target_pub.publish(target_raw_attitude)
        time.sleep(self.Time_between_messages)


################################# LIFT OFF ########################################
	
    def automatic_lift_off_rec(self, thrust, time_flying):
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = "TAKE OFF"
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions taking off: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if self.skip_to_land == True or thrust > self.Liftoff_thrust_maximum:
                print("Going to land")
                break

            elif self.beh_type == "TAKE OFF" and thrust < self.Liftoff_thrust_first: 
                print("Lifting the drone up slowly")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust + self.accumulating_thrust_soft

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance <= 0.4:
                print("The drone is searching the lifting off thrust")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                if self.down_sensor_changed == True:
                    self.down_sensor_changed = False
                    thrust = thrust + 0.005
                    self.change_thrusts(thrust, thrust, thrust)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance >= 0.4 and self.down_sensor_distance <= self.hover_sensor_altitude_min:
                print("The drone is searching the lifting off thrust")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                if self.down_sensor_changed == True:
                    self.down_sensor_changed = False
                    if self.down_sensor_distance_higher == False:
                        thrust = thrust + 0.005
                        self.change_thrusts(thrust, thrust, thrust)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance > self.hover_sensor_altitude_min:
                self.change_thrusts(thrust + 0.015, thrust, thrust - 0.005)
                break
            

    def automatic_lift_off_rec_v2(self, thrust, time_flying):
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = "TAKE OFF"
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions taking off: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if self.skip_to_land == True or thrust > self.Liftoff_thrust_maximum:
                break

            elif self.beh_type == "TAKE OFF" and thrust < 0.54: 
                print("Lifting the drone up slowly")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust + self.accumulating_thrust_soft

            elif self.beh_type == "TAKE OFF" and thrust < 0.56:
                print("The drone is searching the lifting off thrust")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                if self.down_sensor_changed == True:
                    self.down_sensor_changed = False
                    thrust = thrust + 0.005
                    self.change_thrusts(thrust, thrust - 0.0035, thrust - 0.01)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance <= 0.7:
                print("The drone is searching the lifting off thrust")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                if self.down_sensor_changed == True:
                    self.down_sensor_changed = False
                    thrust = thrust + 0.0025
                    self.change_thrusts(thrust, thrust - 0.0035, thrust - 0.01)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance > 0.7:
                self.change_thrusts(thrust, thrust - 0.0035, thrust - 0.01)
                break

################################# HOVER ########################################

    def hover_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)

        thrust = float(self.hover_thrust)
        self.beh_type = 'HOVER'

        reset = 0

        print(recursions)
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions hovering: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)        
            print self.lidarcontrolstate
            self.beh_type = 'HOVER'
            if self.skip_to_land == True:
                break
            
            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)
    
            else:

                obstacle_detected = self.looking_obstacles()

                reset = reset + 1
                if reset >= 50: 

                    self.looking_px4flow()

                    reset = 0

                thrust = self.calculate_thrust()

                self.construct_target_attitude(0,0,0,thrust, 0, 0, 0)
	
        print ("time of hovering has ended")


################################# MOVE RAW ########################################
# Movement in just one direction

#                      /// MOVE RAW ///

    def move_raw(self, time_flying, angle_roll, angle_pitch, angle_yaw): 
        recursions = self.calculate_recursions(time_flying)
        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Liftoff_thrust - self.Landing_thrust
        proportional_thrust = difference_thrust / difference_distance
        print(recursions)
        self.beh_type = 'MOVING'
        thrust = self.Landing_thrust

	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions moving: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)        

            if self.skip_to_land == True:
                break

            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)

            else:
                if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                    distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
                    thrust = self.Liftoff_thrust - distance_more * proportional_thrust + 0.007
                    print('Thrust: ' + str(thrust))
				
                elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                    thrust = self.Landing_thrust
                    print('Thrust: %f    The drone is moving - going down' % (thrust))
				
                elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                    thrust = self.Liftoff_thrust + 0.001
                    print('Thrust: %f    The drone is moving - going up' % (thrust))

                elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max + 0.3 <= self.down_sensor_distance): # We have to go down + change thrusts variables
                    thrust = self.Landing_thrust
                    if self.down_sensor_changed == True:
                        self.down_sensor_changed = False
                        thrust = thrust - 0.001
                        self.change_thrusts(thrust + 0.04, thrust + 0.02, thrust)
                    thrust = self.Landing_thrust
                    print('Thrust: %f    The drone is moving - going down' % (thrust))
				
                elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min - 0.3) and thrust <= self.Liftoff_thrust_maximum: # We have to go up + change thrusts variables
                    thrust = self.Liftoff_thrust 
                    if self.down_sensor_changed == True:
                        self.down_sensor_changed = False
                        thrust = thrust + 0.001
                        self.change_thrusts(thrust, thrust - 0.01, thrust - 0.02)
                    thrust = self.Liftoff_thrust  + 0.001
                    print('Thrust: %f    The drone is moving - going down' % (thrust))

                else:
                    print('Thrust: %f    The drone is moving - no sonar detected' % (thrust))


                self.construct_target_attitude(0,0,0,thrust, angle_roll, angle_pitch, angle_yaw)

################################# SIMPLE MOVEMENTS ########################################
#                      /// RIGHT MOVE ///

    def moving_right_raw_rec(self, time_flying): 

        self.move_raw(time_flying, self.angle_roll_right, 0, 0)      

#                      /// LEFT MOVE ///

    def moving_left_raw_rec(self, time_flying):

        self.move_raw(time_flying, self.angle_roll_left, 0, 0)      

#                      /// FRONT MOVE ///

    def moving_forward_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, self.angle_pitch_forward, 0)       

#                      /// BACK MOVE ///

    def moving_back_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, self.angle_pitch_back, 0)

#                       /// HOVER ///

    def moving_hover_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, 0, 0)

#                      /// DIAGONAL FRONT/RIGHT MOVE ///

    def moving_front_right_raw_rec(self, time_flying): 

        self.move_raw(time_flying, self.angle_roll_right, self.angle_pitch_forward, 0)      

#                      /// DIAGONAL FRONT/LEFT MOVE ///

    def moving_front_left_raw_rec(self, time_flying):

        self.move_raw(time_flying, self.angle_roll_left, self.angle_pitch_forward, 0)      

#                      /// DIAGONAL BACK/RIGHT MOVE ///

    def moving_back_right_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, self.angle_roll_right, self.angle_pitch_back, 0)       

#                      /// DIAGONAL BACK/LEFT MOVE ///

    def moving_back_left_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, self.angle_roll_left, self.angle_pitch_back, 0)

################################# HARD MOVEMENTS ########################################
#                      /// RIGHT MOVE HARD ///

    def moving_right_raw_hard_rec(self, time_flying): 

        self.move_raw(time_flying, self.angle_roll_right_hard, 0, 0)      

#                      /// LEFT MOVE HARD///

    def moving_left_raw_hard_rec(self, time_flying):

        self.move_raw(time_flying, self.angle_roll_left_hard, 0, 0)      

#                      /// FRONT MOVE HARD///

    def moving_forward_raw_hard_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, self.angle_pitch_forward_hard, 0)       

#                      /// BACK MOVE HARD///

    def moving_back_raw_hard_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, self.angle_pitch_back_hard, 0)



#                      /// DIAGONAL FRONT/RIGHT MOVE HARD ///

    def moving_front_right_raw_hard_rec(self, time_flying): 

        self.move_raw(time_flying, self.angle_roll_right_hard, self.angle_pitch_forward_hard, 0)      

#                      /// DIAGONAL FRONT/LEFT MOVE HARD ///

    def moving_front_left_raw_hard_rec(self, time_flying):

        self.move_raw(time_flying, self.angle_roll_left_hard, self.angle_pitch_forward_hard, 0)      

#                      /// DIAGONAL BACK/RIGHT MOVE HARD ///

    def moving_back_right_raw_hard_rec(self, time_flying):
 
        self.move_raw(time_flying, self.angle_roll_right_hard, self.angle_pitch_back_hard, 0)       

#                      /// DIAGONAL BACK/LEFT MOVE HARD ///

    def moving_back_left_raw_hard_rec(self, time_flying):
 
        self.move_raw(time_flying, self.angle_roll_left_hard, self.angle_pitch_back_hard, 0)

################################# COMPLETE MOVEMENTS ########################################
# Each movement consists of three steeps 
# 1. movement in desired direction
# 2. movement in opposite direction 
# 3. hover

#                      /// RIGHT MOVE ///

    def moving_right_rec(self): 
        print("Moving Right")
        self.moving_left_raw_rec(self.move_desired_direction_time)
        self.moving_right_raw_hard_rec(self.move_opposite_direction_time) 
        self.moving_hover_raw_rec(self.move_hover_time)      

#                      /// LEFT MOVE ///

    def moving_left_rec(self):
        print("Moving Left")
        self.moving_left_raw_rec(self.move_desired_direction_time)
        self.moving_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       

#                      /// FRONT MOVE ///

    def moving_forward_rec(self):
        print("Moving Front") 
        self.moving_forward_raw_rec(self.move_desired_direction_time)
        self.moving_back_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       

#                      /// BACK MOVE ///

    def moving_back_rec(self):
        print("Moving Back")
        self.moving_back_raw_rec(self.move_desired_direction_time)
        self.moving_forward_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       



#                      /// FRONT / RIGHT MOVE ///

    def moving_front_right_rec(self): 
        print("Moving Front and Right")
        self.moving_front_right_raw_rec(self.move_desired_direction_time)
        self.moving_back_left_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)     

#                      /// FRONT / LEFT MOVE ///

    def moving_front_left_rec(self):
        print("Moving Front and Left")
        self.moving_front_left_raw_rec(self.move_desired_direction_time)
        self.moving_back_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)
      

#                      /// BACK / RIGHT MOVE ///

    def moving_back_right_rec(self):
        print("Moving Back and Right") 
        self.moving_back_right_raw_rec(self.move_desired_direction_time)
        self.moving_front_left_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time) 
      

#                      /// BACK / LEFT MOVE ///

    def moving_back_left_rec(self):
        print("Moving Back and Left")
        self.moving_back_left_raw_rec(self.move_desired_direction_time)
        self.moving_front_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time) 



################################# COMPLETE MOVEMENTS HARD ########################################
# Each movement consists of three steeps 
# 1. movement in desired direction
# 2. movement in opposite direction 
# 3. hover

#                      /// RIGHT MOVE ///

    def moving_right_rec_hard(self): 
        print("Moving Right")
        self.moving_left_raw_hard_rec(self.move_desired_direction_time)
        self.moving_right_raw_hard_rec(self.move_opposite_direction_time) 
        self.moving_hover_raw_rec(self.move_hover_time)      

#                      /// LEFT MOVE ///

    def moving_left_rec_hard(self):
        print("Moving Left")
        self.moving_left_raw_hard_rec(self.move_desired_direction_time)
        self.moving_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       

#                      /// FRONT MOVE ///

    def moving_forward_rec_hard(self):
        print("Moving Front") 
        self.moving_forward_hard_raw_rec(self.move_desired_direction_time)
        self.moving_back_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       

#                      /// BACK MOVE ///

    def moving_back_rec_hard(self):
        print("Moving Back")
        self.moving_back_raw_hard_rec(self.move_desired_direction_time)
        self.moving_forward_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       



#                      /// FRONT / RIGHT MOVE ///

    def moving_front_right_rec_hard(self): 
        print("Moving Front and Right")
        self.moving_front_right_raw_hard_rec(self.move_desired_direction_time)
        self.moving_back_left_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)     

#                      /// FRONT / LEFT MOVE ///

    def moving_front_left_rec_hard(self):
        print("Moving Front and Left")
        self.moving_front_left_raw_hard_rec(self.move_desired_direction_time)
        self.moving_back_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)
      

#                      /// BACK / RIGHT MOVE ///

    def moving_back_right_rec_hard(self):
        print("Moving Back and Right") 
        self.moving_back_right_raw_hard_rec(self.move_desired_direction_time)
        self.moving_front_left_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time) 
      

#                      /// BACK / LEFT MOVE ///

    def moving_back_left_rec_hard(self):
        print("Moving Back and Left")
        self.moving_back_left_raw_hard_rec(self.move_desired_direction_time)
        self.moving_front_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)





################################# LANDING ########################################
	
    def landing_rec(self):                       # Landing phase
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Max_time_landing)
        print(recursions)
        thrust = self.hover_thrust
        for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions landing: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if self.down_sensor_distance <= self.landing_sensor_altitude_min:
                break

            elif thrust > self.Landing_thrust: 
                print('Thrust: ' + str(thrust) + '   Landing the drone down slowly"')
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust - self.accumulating_thrust_soft

            elif thrust <= self.Landing_thrust: 
                thrust = self.Landing_thrust
                print('Thrust: ' + str(thrust) + '   Landing the drone down slowly"')
                self.construct_target_attitude(0,0,0,thrust)
				
        self.secure_landing_phase_rec2()

# Secure landing part (it keep sending orders of landing during a few seconds just to not disconnect the drone when the sensor stops working - 30cm)
    def secure_landing_phase_rec2(self):           
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Secure_time_landing)
        thrust = self.Landing_thrust	
        for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions landing: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if thrust <= 0:
                break

            if i < 1000:
                thrust = self.Landing_thrust
                print('Thrust: ' + str(thrust) + '   Smooth landing')
                self.construct_target_attitude(0,0,0,thrust)

            else: 
                print('Thrust: ' + str(thrust) + '   Landing')
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust - self.accumulating_thrust_soft

        for i in range(20):
            
            self.arm_state = self.disarm()    # disarms the drone
            time.sleep(0.1)

    def secure_landing_phase_rec(self):           # Secure landing part - last cm - not used, just another idea to land the drone
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Secure_time_landing)	
        thrust = self.hover_thrust
        for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions secure landing: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if thrust <= 0:
                break

            elif i < 200:
                thrust = self.hover_thrust
                print('Thrust: ' + str(thrust) + '   Secure hover before landing')
                self.construct_target_attitude(0,0,0,thrust)

            elif i < 700:
                thrust = self.hover_thrust - 0.000002
                print('Thrust: ' + str(thrust) + '   Smooth landing')
                self.construct_target_attitude(0,0,0,thrust)

            else: 
                thrust = thrust - 0.000002
                print('Thrust: ' + str(thrust) + '   Landing')
                self.construct_target_attitude(0,0,0,thrust)
        for i in range(20):
            
            self.arm_state = self.disarm()    # disarms the drone
            time.sleep(0.1)	

################################# OBSTACLE AVOIDANCE ########################################

    def lidar_obstacle_detection_callback(self, msg):
        self.lidarcontrolstate = msg.data 
        
#----------------------change modes----------------------------

    def modechnge(self):
        #rospy.init_node("offboard_node")
        if self.flightModeService(custom_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False        


    def modechnge3(self):
        #rospy.init_node("offboard_node")
        if self.takeoffService():
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False 

    def modechnge4(self):
        #rospy.init_node("offboard_node")
        if self.flightModeService(custom_mode='STABILIZED'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False  

    def modechnge5(self):
        #rospy.init_node("offboard_node")
        if self.landService():
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False 

    def modechnge6(self):
        #rospy.init_node("offboard_node")
        if self.flightModeService(custom_mode='POSCTL'):
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

        #self.cur_target_pose = self.construct_target(self.init_x, self.init_y, self.init_z, self.current_heading)
        #self.local_target_pub.publish(self.cur_target_pose)

        time.sleep(2)

        for i in range(20):
            
            self.arm_state = self.arm()    # arms the drone
            self.construct_target_attitude()
            self.offboard_state = self.modechnge()
            time.sleep(0.1)
            
 

 
#####################################################################################################            
if __name__ == '__main__':

    try:
        Gpio_start().start()
        time.sleep(3.5)
        print("Waiting for Advandiptera brain")
        arm = Arming_Modechng()

        # Arming + liftoff + hover
        arm.start()
        if arm.down_sensor_distance != None:
            #arm.lift_off_rec(arm.init_thrust, arm.Liftoff_time)
            #arm.automatic_lift_off_rec(arm.init_thrust, arm.Liftoff_time)
            arm.automatic_lift_off_rec_v2(arm.init_thrust, arm.Liftoff_time)

            arm.hover_rec(arm.hover_time)
            #arm.automatic_hover_rec(arm.hover_time)
       
            # Moving
            #arm.moving_right_rec()
            #arm.moving_left_rec()
            #arm.moving_back_rec()

            #arm.moving_forward_rec(arm.Moving_time)
            #arm.hover_rec(2)
            #arm.moving_left_rec(arm.Moving_time)
            #arm.hover_rec(2)
            #arm.moving_back_rec(arm.Moving_time)
            #arm.hover_rec(2)
            #arm.moving_right_rec(arm.Moving_time)

            # Hover + landing
            #arm.hover_rec(5)
            arm.landing_rec()
        else:
            print("No sonar detected, disarming")

    except rospy.ROSInterruptException: pass
