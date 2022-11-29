#!/usr/bin/env python

import PyLidar2
import time # Time module
import numpy as np
import math
import thread
from sensor_msgs.msg import Range
import rospy
from sensor_msgs.msg import LaserScan
import yaml
from std_msgs.msg import String

class Lidar_diptera():
    def __init__(self):
        self.port = "/dev/ttyUSB1" #linux
        self.Obj = PyLidar2.YdLidarX4(self.port) #PyLidar2.your_version_of_lidar(port,chunk_size)
        self.gen = self.Obj.StartScanning()
        rospy.init_node('laser_scan_obstacle_finder')

        self.x_pointcloud = []
        self.y_pointcloud = []
        for _ in range(360):
            self.x_pointcloud.append(0)
            self.y_pointcloud.append(0)

        self.right_distance = 50000
        self.left_distance = 50000
        self.front_distance = 50000
        self.back_distance = 50000
        self.front_right_distance = 50000
        self.front_left_distance = 50000
        self.back_right_distance = 50000
        self.back_left_distance = 50000

        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():

                if key == "Avoiding_obstacle_distance_min":
                    self.Avoiding_obstacle_distance_min = value

        self.lidar_avoidance_pub = rospy.Publisher('/custom/lidaravoidance', String, queue_size=10) 

    def spin(self):
        if(self.Obj.Connect()):
            print(self.Obj.GetDeviceInfo())
            t = time.time() # start time
            while (time.time() - t) < 1000000: #scan for 30 seconds
                print(self.gen.next())
                time.sleep(0.5)
            self.Obj.StopScanning()
            self.Obj.Disconnect()
        else:
            print("Error connecting to device")

    def lazer_msg_constructor(self,topic_name ,lazer_frame ,distance ,angle):
        r = Range()
        r.header.stamp = rospy.Time.now()
        r.header.frame_id = lazer_frame
        r.radiation_type = 0
        r.field_of_view = 0.8
        r.min_range = 0
        r.max_range = 10000
        r.radiation_type = 1
        r.range = distance
        scan_pub = rospy.Publisher(topic_name, Range, queue_size=10)
        scan_pub.publish(r)
        time.sleep(0.05)
        #r = rospy.Rate(2)

        #scan = LaserScan()
        #current_time = rospy.Time.now()
        #scan.header.stamp = current_time
        #scan.header.frame_id = lazer_frame
        #scan.angle_min = -1.57
        #scan.angle_max = 1.57
        #scan.angle_increment = 3.14 / 360
        #scan.time_increment = (1.0 / 4) / 360
        #scan.range_min = 0.0
        #scan.range_max = 1000.0
        #scan.ranges = []
        #scan.ranges[angle] = distance
        #scan_pub.publish(scan)


    def forward_obs_det(self,data):
        value = 0
        count = 0
        for angle in range(275,285,1): # 225 degrees to 315 degrees
           if (300 < data[angle] ):
               #print("obstacle infront --> move back\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1

        if value != 0:
            value = value / count
        else: 
            value = 50000
        self.front_distance = value
        obstacle_direction_frame = "Front Obstacle"
        obstacle_direction_topic = "/Front_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def backward_obs_det(self,data):
        value = 0
        count = 0
        for angle in range(85,95,1): # 45 degrees to 135 degrees
            if (300 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1
        if value != 0:
            value = value / count
        else: 
            value = 50000
        self.back_distance = value
        obstacle_direction_frame = "Back Obstacle"
        obstacle_direction_topic = "/Back_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def left_obs_det(self,data):
        value = 0
        count = 0
        for angle in range(175,185,2): # 135 degrees to 225 degrees
            if (300 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1

        if value != 0:
            value = value / count
        else: 
            value = 50000
        self.left_distance = value
        obstacle_direction_frame = "Left Obstacle"
        obstacle_direction_topic = "/Left_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def right_obs_det(self,data):
        value = 0
        count = 0
        for angle in range(355,359,1): # 315 degrees to 45 degrees, divided in two steps
            if (300 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1

        for angle in range(0,5,1):
            if (300 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1

        if value != 0:
            value = value / count
        else: 
            value = 50000
        self.right_distance = value
        obstacle_direction_frame = "Right Obstacle"
        obstacle_direction_topic = "/Right_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

############################# DIAGONALS ###############################

    def forward_right_obs_det(self,data):
        value = 0
        count = 0
        for angle in range(310,320,1): 
           if (300 < data[angle] ):
               #print("obstacle infront --> move back\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1

        if value != 0:
            value = value / count
        else: 
            value = 50000
        self.front_right_distance = value
        obstacle_direction_frame = "Front Right Obstacle"
        obstacle_direction_topic = "/Front_Right_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def forward_left_obs_det(self,data):
        value = 0
        count = 0
        for angle in range(220,230,1): # 45 degrees to 135 degrees
            if (300 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1
        if value != 0:
            value = value / count
        else: 
            value = 50000
        self.front_left_distance = value
        obstacle_direction_frame = "Front Left Obstacle"
        obstacle_direction_topic = "/Front_Left_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def back_right_obs_det(self,data):
        value = 0
        count = 0
        for angle in range(40,50,2): # 135 degrees to 225 degrees
            if (300 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1

        if value != 0:
            value = value / count
        else: 
            value = 50000
        self.back_right_distance = value
        obstacle_direction_frame = "Back Right Obstacle"
        obstacle_direction_topic = "/Back_Right_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def back_left_obs_det(self,data):
        value = 0
        count = 0
        for angle in range(130,140,1): # 315 degrees to 45 degrees, divided in two steps
            if (300 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
               value = value + data[angle]
               count = count + 1

        if value != 0:
            value = value / count
        else: 
            value = 50000
        self.back_left_distance = value
        obstacle_direction_frame = "Back Left Obstacle"
        obstacle_direction_topic = "/Back_Left_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)



    def point_cloud(self):
        if(self.Obj.Connect()):
            t = time.time()
            while ((time.time() - t) < 1000000):
                data = self.gen.next()
                #print data
                self.forward_obs_det(data)
                self.backward_obs_det(data)
                self.left_obs_det(data)
                self.right_obs_det(data)
                self.forward_right_obs_det(data)
                self.forward_left_obs_det(data)
                self.back_right_obs_det(data)
                self.back_left_obs_det(data)

                # DIAGONALS
                if self.front_right_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Left
                    self.lidar_avoidance_pub.publish(String("Front Right"))

                elif self.front_left_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Right
                    self.lidar_avoidance_pub.publish(String("Front Left"))

                elif self.back_right_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Left
                    self.lidar_avoidance_pub.publish(String("Back Right"))

                elif self.back_left_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Right
                    self.lidar_avoidance_pub.publish(String("Back Left"))



                # LINEALS
                elif self.front_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Front
                    self.lidar_avoidance_pub.publish(String("Front"))

                elif self.back_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Back
                    self.lidar_avoidance_pub.publish(String("Back"))

                elif self.left_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Left
                    self.lidar_avoidance_pub.publish(String("Left"))

                elif self.right_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Right
                    self.lidar_avoidance_pub.publish(String("Right"))
     
                # NO OBSTACLE
                else:
                    self.lidar_avoidance_pub.publish(String("No Obstacle"))

               

ld = Lidar_diptera()
#ld.spin()
ld.point_cloud()
