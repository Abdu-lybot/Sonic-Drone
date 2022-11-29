import rospy
from quaternion import Quaternion
from mavros_msgs.msg import GlobalPositionTarget, State, AttitudeTarget
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
import time
import yaml
import math

class Move_Drone():

    def __init__(self):
        #rospy.init_node("Move_drone_node")

        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.cb_imu)
        self.attitude_target_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)


    def construct_target(self, body_x = 0, body_y = 0, body_z = 0, thrust = 1):
        target_raw_attitude = AttitudeTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_attitude.header.stamp = rospy.Time.now()
        #target_raw_attitude.orientation. = self.imu.orientation
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE \
                                    + AttitudeTarget.IGNORE_ATTITUDE

        #target_raw_attitude.body_rate.x = body_x # ROLL_RATE
        #target_raw_attitude.body_rate.y = body_y # PITCH_RATE
        #target_raw_attitude.body_rate.z = body_z # YAW_RATE
        target_raw_attitude.thrust = thrust
        return target_raw_attitude

    def cb_imu(self, msg):
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True

    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            self.q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad =self.q_.yaw_pitch_roll[0]
        return rotate_z_rad

    def waiting_initialization(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

    def test(self):
        self.cur_target_attitude = self.construct_target()
        self.attitude_target_pub.publish(self.cur_target_attitude)   

    
    # Moves to a determinate location
    def start(self, distance, body_x, body_y, body_z, thrust):
        self.waiting_initialization()
        timetowait = distance/thrust
        self.cur_target_attitude = self.construct_target(body_x, body_y, body_z, thrust)
        self.attitude_target_pub.publish(self.cur_target_attitude)
        time.sleep(timetowait)
        self.cur_target_attitude = self.construct_target(0,0,0,0)
        self.attitude_target_pub.publish(self.cur_target_attitude) 

    # Moves a determinate distance
    def move_in_x(self, distance, thrust, angle):
        self.waiting_initialization()
        timetowait = distance/thrust
        self.cur_target_attitude = self.construct_target( 0, 0, angle * math.pi / 180.0, thrust)
        self.attitude_target_pub.publish(self.cur_target_attitude) 
        time.sleep(timetowait)
        self.cur_target_attitude = self.construct_target(0,0,0,0)
        self.attitude_target_pub.publish(self.cur_target_attitude)

    def move_in_y(self, distance, thrust, angle):
        self.waiting_initialization()
        timetowait = distance/thrust
        self.cur_target_attitude = self.construct_target(0, 0, angle * math.pi / 180.0, thrust)
        self.attitude_target_pub.publish(self.cur_target_attitude) 
        time.sleep(timetowait)
        self.cur_target_attitude = self.construct_target(0,0,0,0)
        self.attitude_target_pub.publish(self.cur_target_attitude)

    def move_in_z(self, distance, thrust, angle):
        self.waiting_initialization()
        timetowait = distance/thrust
        self.cur_target_attitude = self.construct_target(0, angle * math.pi / 180.0, 0, thrust)
        self.attitude_target_pub.publish(self.cur_target_attitude) 
        time.sleep(timetowait)
        self.cur_target_attitude = self.construct_target(0,0,0,0)
        self.attitude_target_pub.publish(self.cur_target_attitude)
    
    # Moves in a determined direction        
    def moving_forward(self, distance, thrust = 0.2):
        self.move_in_x(distance, thrust, 0)

    def moving_back(self, distance, thrust = 0.2):
        self.move_in_x(distance, thrust, 180)

    def moving_left(self, distance, thrust = 0.2):
        self.move_in_y(distance, thrust, 90)

    def moving_right(self, distance, thrust = 0.2):
        self.move_in_y(-distance, thrust, -90)

    def moving_up(self, distance, thrust = 0.2):
        self.move_in_z(distance, thrust, 90)

    def moving_down(self, distance, thrust = 0.2):
        self.move_in_z(-distance, thrust, -90)


