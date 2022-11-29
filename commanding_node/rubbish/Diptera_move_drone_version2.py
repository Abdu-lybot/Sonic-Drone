import rospy
from quaternion import Quaternion
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
import time
import yaml


class Move_Drone():

    def __init__(self):
        #rospy.init_node("Move_drone_node")
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.cb_imu)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
 
        self.velocity_value = 0.2
        self.current_heading = None
        self.waiting_initialization()


    def construct_target(self, x, y, z, yaw, x_vel = 0.2, y_vel = 0.2, z_vel = 0.2, yaw_rate=1):
        target_raw_pose = PositionTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_pose.header.stamp = rospy.Time.now()
        target_raw_pose.coordinate_frame = 9
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z
        target_raw_pose.velocity.x = x_vel
        target_raw_pose.velocity.y = y_vel
        target_raw_pose.velocity.z = z_vel

        target_raw_pose.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose


    def cb_local_pose(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg


    def cb_imu(self, msg):
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

    def waiting_initialization(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
    
    # Moves to a determinate location
    def start(self, x, y, z, yaw, x_vel = 0.2, y_vel = 0.2, z_vel = 0.2, yaw_rate = 1):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(x, y, z, yaw, x_vel, y_vel, z_vel, yaw_rate)
        self.local_target_pub.publish(self.cur_target_pose) 

    # Moves a determinate distance
    def move_in_x(self, distance):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(distance, 0, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)
        timetowait = distance/self.velocity_value
        time.sleep(timetowait)
        self.cur_target_pose = self.construct_target(0, 0, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)  

    def move_in_y(self, distance):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(0, distance, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 
        timetowait = distance/self.velocity_value
        time.sleep(timetowait)
        self.cur_target_pose = self.construct_target(0, 0, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 

    def move_in_z(self, distance):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(0, 0, self.local_pose.pose.position.z + distance, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)
        timetowait = distance/self.velocity_value
        time.sleep(timetowait)
        self.cur_target_pose = self.construct_target(0, 0, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)  
    
    # Moves in a determined direction        
    def moving_forward(self, distance):
        self.move_in_x(distance)

    def moving_back(self, distance):
        self.move_in_x(distance)

    def moving_left(self, distance):
        self.move_in_y(-distance)

    def moving_right(self, distance):
        self.move_in_y(-distance)

    def moving_up(self, distance):
        self.move_in_z(distance)

    def moving_down(self, distance):
        self.move_in_z(-distance)


########################################TEST#########################################
    def test_x(self):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(1, 0, 0, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)

    def test_y(self):
        self.cur_target_pose = self.construct_target(0, 1, 0, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)

    def test_z(self):
        self.cur_target_pose = self.construct_target(0, 0, 1, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)


    # Moves to a determinate location
'''
    def move_to_x(self, x_location):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(x_location, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 

    def move_to_y(self, y_location):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, y_location, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 

    def move_to_z(self, z_location):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, z_location, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 
'''






