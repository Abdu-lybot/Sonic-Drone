#!/usr/bin/env python

import PyLidar2
import time # Time module
import numpy as np
import math
import thread
from sensor_msgs.msg import Range
import rospy
from sensor_msgs.msg import LaserScan



class Lidar_diptera():
    def __init__(self):
        self.port = "/dev/ttyUSB0" #linux
        self.Obj = PyLidar2.YdLidarX4(self.port) #PyLidar2.your_version_of_lidar(port,chunk_size)
        self.gen = self.Obj.StartScanning()
        rospy.init_node('laser_scan_obstacle_finder')

        self.x_pointcloud = []
        self.y_pointcloud = []
        for _ in range(360):
            self.x_pointcloud.append(0)
            self.y_pointcloud.append(0)

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
        value = 10000
        for angle in range(235,305,5): # 225 degrees to 315 degrees
           if (500 < data[angle] ):
               #print("obstacle infront --> move back\n",angle,data[angle])
               if (int(data[angle])/10 < value ):
                   value = int(data[angle])/10

        obstacle_direction_frame = "Front Obstacle"
        obstacle_direction_topic = "/Front_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def backward_obs_det(self,data):
        value = 10000
        for angle in range(55,125,5): # 45 degrees to 135 degrees
            if (500 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
                if (int(data[angle])/10 < value ):
                   value = int(data[angle])/10

        obstacle_direction_frame = "Back Obstacle"
        obstacle_direction_topic = "/Back_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def left_obs_det(self,data):
        value = 10000
        for angle in range(145,215,5): # 135 degrees to 225 degrees
            if (500 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
                if (int(data[angle])/10 < value ):
                   value = int(data[angle])/10

        obstacle_direction_frame = "Left Obstacle"
        obstacle_direction_topic = "/Left_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def right_obs_det(self,data):
        value = 10000
        for angle in range(325,355,5): # 315 degrees to 45 degrees, divided in two steps
            if (500 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
                if (int(data[angle])/10 < value ):
                   value = int(data[angle])/10

        for angle in range(5,35,5):
            if (500 < data[angle] ):
                #print("obstacle back --> move front\n",angle,data[angle])
                if (int(data[angle])/10 < value ):
                   value = int(data[angle])/10

        obstacle_direction_frame = "Right Obstacle"
        obstacle_direction_topic = "/Right_Obstacle"
        self.lazer_msg_constructor(obstacle_direction_topic,obstacle_direction_frame, np.float_(value) ,angle)

    def point_cloud(self):
        if(self.Obj.Connect()):
            t = time.time()
            while ((time.time() - t) < 1000000):
                data = self.gen.next()
                print data
                self.forward_obs_det(data)
                self.backward_obs_det(data)
                self.left_obs_det(data)
                self.right_obs_det(data)
                '''
                try:
                    thread.start_new_thread(self.forward_obs_det, (data, ))
                    thread.start_new_thread(self.backward_obs_det, (data, ))
                    thread.start_new_thread(self.left_obs_det, (data, ))
                    thread.start_new_thread(self.right_obs_det, (data, ))
                except:
                    print ("error")
                '''
                

ld = Lidar_diptera()
#ld.spin()
ld.point_cloud()
