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

        self.right_distance = 5000
        self.left_distance = 5000
        self.front_distance = 5000
        self.back_distance = 5000
        self.front_right_distance = 5000
        self.front_left_distance = 5000
        self.back_right_distance = 5000
        self.back_left_distance = 5000

        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():

                if key == "Avoiding_obstacle_distance_min":
                    self.Avoiding_obstacle_distance_min = value
                if key == "Avoiding_obstacle_distance_hard_min":
                    self.Avoiding_obstacle_distance_hard_min = value

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
        value1 = 0
        count1 = 0
        value2 = 0
        count2 = 0

        for angle in range(258,278,1): 
           if (300 < data[angle] ):
               if data[angle] > 5000:
                   value1 = value1 + 5000
               else:
                   value1 = value1 + data[angle]
               count1 = count1 + 1

        for angle in range(272,282,1): 
           if (300 < data[angle] ):
               if data[angle] > 5000:
                   value2 = value2 + 5000
               else:
                   value2 = value2 + data[angle]
               count2 = count2 + 1

        if value1 != 0:
            value1 = value1 / count1
        else: 
            value1 = 5000
        if value2 != 0:
            value2 = value2 / count2
        else: 
            value2 = 5000

        if value1 < value2:
            value = value1
        else:
            value = value2

        self.front_distance = value
        obstacle_direction_frame = "Front Obstacle"
        obstacle_direction_topic = "/Front_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def backward_obs_det(self,data):
        value1 = 0
        count1 = 0
        value2 = 0
        count2 = 0

        for angle in range(78,88,1): 
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value1 = value1 + 5000
               else:
                   value1 = value1 + data[angle]
               count1 = count1 + 1

        for angle in range(92,102,1):
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value2 = value2 + 5000
               else:
                   value2 = value2 + data[angle]
               count2 = count2 + 1

        if value1 != 0:
            value1 = value1 / count1
        else: 
            value1 = 5000
        if value2 != 0:
            value2 = value2 / count2
        else: 
            value2 = 5000

        if value1 < value2:
            value = value1
        else:
            value = value2

        self.back_distance = value
        obstacle_direction_frame = "Back Obstacle"
        obstacle_direction_topic = "/Back_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def left_obs_det(self,data):
        value1 = 0
        count1 = 0
        value2 = 0
        count2 = 0
        for angle in range(168,178,2):
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value1 = value1 + 5000
               else:
                   value1 = value1 + data[angle]
               count1 = count1 + 1

        for angle in range(182,192,2): 
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value2 = value2 + 5000
               else:
                   value2 = value2 + data[angle]
               count2 = count2 + 1

        if value1 != 0:
            value1 = value1 / count1
        else: 
            value1 = 5000
        if value2 != 0:
            value2 = value2 / count2
        else: 
            value2 = 5000

        if value1 < value2:
            value = value1
        else:
            value = value2
        self.left_distance = value
        obstacle_direction_frame = "Left Obstacle"
        obstacle_direction_topic = "/Left_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def right_obs_det(self,data):
        value1 = 0
        count1 = 0
        value2 = 0
        count2 = 0
        for angle in range(348,358,1): # 315 degrees to 45 degrees, divided in two steps
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value1 = value1 + 5000
               else:
                   value1 = value1 + data[angle]
               count1 = count1 + 1

        for angle in range(2,12,1):
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value2 = value2 + 5000
               else:
                   value2 = value2 + data[angle]
               count2 = count2 + 1

        if value1 != 0:
            value1 = value1 / count1
        else: 
            value1 = 5000
        if value2 != 0:
            value2 = value2 / count2
        else: 
            value2 = 5000

        if value1 < value2:
            value = value1
        else:
            value = value2

        self.right_distance = value
        obstacle_direction_frame = "Right Obstacle"
        obstacle_direction_topic = "/Right_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

############################# DIAGONALS ###############################
    
    def forward_right_obs_det(self,data):
        value1 = 0
        count1 = 0
        value2 = 0
        count2 = 0
        for angle in range(303,313,1): 
           if (300 < data[angle] ):
               if data[angle] > 5000:
                   value1 = value1 + 5000
               else:
                   value1 = value1 + data[angle]
               count1 = count1 + 1

        for angle in range(317,327,1):
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value2 = value2 + 5000
               else:
                   value2 = value2 + data[angle]
               count2 = count2 + 1

        if value1 != 0:
            value1 = value1 / count1
        else: 
            value1 = 5000
        if value2 != 0:
            value2 = value2 / count2
        else: 
            value2 = 5000

        if value1 < value2:
            value = value1
        else:
            value = value2

        self.front_right_distance = value
        obstacle_direction_frame = "Front Right Obstacle"
        obstacle_direction_topic = "/Front_Right_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def forward_left_obs_det(self,data):
        value1 = 0
        count1 = 0
        value2 = 0
        count2 = 0
        for angle in range(213,223,1):
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value1 = value1 + 5000
               else:
                   value1 = value1 + data[angle]
               count1 = count1 + 1

        for angle in range(227,237,1):
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value2 = value2 + 5000
               else:
                   value2 = value2 + data[angle]
               count2 = count2 + 1

        if value1 != 0:
            value1 = value1 / count1
        else: 
            value1 = 5000
        if value2 != 0:
            value2 = value2 / count2
        else: 
            value2 = 5000

        if value1 < value2:
            value = value1
        else:
            value = value2

        self.front_left_distance = value
        obstacle_direction_frame = "Front Left Obstacle"
        obstacle_direction_topic = "/Front_Left_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def back_right_obs_det(self,data):
        value1 = 0
        count1 = 0
        value2 = 0
        count2 = 0

        for angle in range(33,43,2): 
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value1 = value1 + 5000
               else:
                   value1 = value1 + data[angle]
               count1 = count1 + 1

        for angle in range(47,57,1):
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value2 = value2 + 5000
               else:
                   value2 = value2 + data[angle]
               count2 = count2 + 1

        if value1 != 0:
            value1 = value1 / count1
        else: 
            value1 = 5000
        if value2 != 0:
            value2 = value2 / count2
        else: 
            value2 = 5000

        if value1 < value2:
            value = value1
        else:
            value = value2

        self.back_right_distance = value
        obstacle_direction_frame = "Back Right Obstacle"
        obstacle_direction_topic = "/Back_Right_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def back_left_obs_det(self,data):
        value1 = 0
        count1 = 0
        value2 = 0
        count2 = 0

        for angle in range(123,133,1): 
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value1 = value1 + 5000
               else:
                   value1 = value1 + data[angle]
               count1 = count1 + 1

        for angle in range(137,147,1):
            if (300 < data[angle] ):
               if data[angle] > 5000:
                   value2 = value2 + 5000
               else:
                   value2 = value2 + data[angle]
               count2 = count2 + 1

        if value1 != 0:
            value1 = value1 / count1
        else: 
            value1 = 5000
        if value2 != 0:
            value2 = value2 / count2
        else: 
            value2 = 5000

        if value1 < value2:
            value = value1
        else:
            value = value2

        self.back_left_distance = value
        obstacle_direction_frame = "Back Left Obstacle"
        obstacle_direction_topic = "/Back_Left_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)
    

############################### AVOIDING OBSTACLE #################################

    def point_cloud(self):
        if(self.Obj.Connect()):
            t = time.time()
            #while ((time.time() - t) < 1000000):
            while not rospy.is_shutdown():
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
                if self.front_right_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Front/Right

                    #if self.front_right_distance <= self.Avoiding_obstacle_distance_hard_min:
                    #    self.lidar_avoidance_pub.publish(String("Front Right Hard"))
                    #else: 
                    self.lidar_avoidance_pub.publish(String("Front Right"))                        

                elif self.front_left_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Front/Left
                    #if self.front_left_distance <= self.Avoiding_obstacle_distance_hard_min:
                    #    self.lidar_avoidance_pub.publish(String("Front Left Hard"))
                    #else:
                    self.lidar_avoidance_pub.publish(String("Front Left"))

                if self.back_right_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Back/Right
                    #if self.back_right_distance <= self.Avoiding_obstacle_distance_hard_min:
                    #    self.lidar_avoidance_pub.publish(String("Back Right Hard"))
                    #else:
                    self.lidar_avoidance_pub.publish(String("Back Right"))

                if self.back_left_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Back/Left
                    #if self.back_left_distance <= self.Avoiding_obstacle_distance_hard_min:
                    #    self.lidar_avoidance_pub.publish(String("Back Left Hard"))
                    #else:
                    self.lidar_avoidance_pub.publish(String("Back Left"))

                    
                elif self.front_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Front
                    #if self.front_distance <= self.Avoiding_obstacle_distance_hard_min:
                    #    self.lidar_avoidance_pub.publish(String("Front Hard"))
                    #else:
                    self.lidar_avoidance_pub.publish(String("Front"))

                elif self.back_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Back
                    #if self.back_distance <= self.Avoiding_obstacle_distance_hard_min:
                    #    self.lidar_avoidance_pub.publish(String("Back Hard"))
                    #else:
                    self.lidar_avoidance_pub.publish(String("Back"))

                elif self.left_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Left
                    #if self.left_distance <= self.Avoiding_obstacle_distance_hard_min:
                    #    self.lidar_avoidance_pub.publish(String("Left Hard"))
                    #else:
                    self.lidar_avoidance_pub.publish(String("Left"))

                elif self.right_distance <= self.Avoiding_obstacle_distance_min:  # Obstacle Right
                    #if self.right_distance <= self.Avoiding_obstacle_distance_hard_min:
                    #    self.lidar_avoidance_pub.publish(String("Right Hard"))
                    #else:
                    self.lidar_avoidance_pub.publish(String("Right"))
                    
     
                # NO OBSTACLE
                else:
                    self.lidar_avoidance_pub.publish(String("No Obstacle"))

               
if __name__ == '__main__':
    try:
        ld = Lidar_diptera()
        #ld.spin()
        ld.point_cloud()

    except rospy.ROSInterruptException: pass

