#!/usr/bin/env python

import rospy
import numpy as np
from database_robot.msg import Reading
from database_robot.msg import ReadingList
from std_srvs.srv import Trigger
import time
import csv


class Stigmergy:

    def __init__(self):

	rospy.Subscriber("/rfid_readings_list", ReadingList, self.reading_list_cb, queue_size=1)

	self.reset_memory()

    def reading_list_cb(self, msg):
        self.last_read = msg.readings

	for read in self.last_read:
	    if read.antenna_port == "4":
	        self.last_front_read.append(read)
	    elif read.antenna_port == "2":
	        self.last_right_read.append(read)
	    elif read.antenna_port == "1":
		self.last_back_read.append(read)
	    elif read.antenna_port == "3":
	        self.last_left_read.append(read)
	    self.read_tags[read.epc] = 0
	
    def count_new_reads(self, direction, read_list):
	count=0
	for read in read_list:
	    epc = self.is_new(direction, read)
	    if epc is not None:
		count+=1
		self.used_tags_now[direction][epc]=0

	return count
	        
    def is_new(self, direction, read):
	epc = read.epc
	if epc not in self.used_tags.keys() and epc not in self.used_tags_now[direction].keys():
	    return epc
	else:
	    return None

    def winner_antenna(self, angle):
	angle = angle*180/np.pi
	if -45 < angle < 45:
            return "front"
        elif 45 < angle < 135:
            return "left"
        elif -135 < angle < -45:
            return "right"
        else:
            return "back"


    def update_angle(self):

	front_num = self.count_new_reads("front", self.last_front_read)
	right_num = self.count_new_reads("right", self.last_right_read)
	back_num = self.count_new_reads("back", self.last_back_read)
	left_num = self.count_new_reads("left", self.last_left_read)
	
	total_detections = float(front_num+back_num+left_num+right_num)

	if total_detections == 0:
            self.start_reading_again()
	    return None

	front_w = front_num/total_detections
	right_w = right_num/total_detections
	back_w = back_num/total_detections
	left_w = left_num/total_detections
	direction_vector = np.array([0.0,1.0])*front_w + np.array([1.0,0.0])*right_w + np.array([0.0,-1.0])*back_w + np.array([-1.0,0.0])*left_w
        norm = np.linalg.norm(direction_vector)
	
	if norm == 0:
	    self.start_reading_again()
	    return None

	direction_vector_normalized = direction_vector / norm
	direction_angle = np.arccos(direction_vector_normalized[1])
	if right_w > left_w:
	    direction_angle*=-1

	#direction = self.winner_antenna(direction_angle)
	#self.used_tags.update(self.used_tags_now[direction])

	self.used_tags.update(self.used_tags_now["front"])
	self.used_tags.update(self.used_tags_now["right"])
	self.used_tags.update(self.used_tags_now["back"])
	self.used_tags.update(self.used_tags_now["left"])

	#print("\n--STIGMERGY--")
        #print("Total considered tags: ")
	#print(len(self.used_tags))
	#print("On this call: ")
	#print(len(self.used_tags_now))
	#print("Number of New Detections:")
	print(front_num)
	print(right_num)
	print(back_num)
	print(left_num)
	#print(total_detections)
	#print(direction_angle , direction_angle*180/np.pi)

	self.start_reading_again()

        return direction_angle

    def reset_memory(self):

	self.start_reading_again()
	self.used_tags = dict()
	self.read_tags = dict()

    def start_reading_again(self):
	self.last_read = list()

	self.last_front_read = list()
	self.last_right_read = list()
	self.last_back_read = list()
	self.last_left_read = list()

	self.used_tags_now = {"front":dict(), "right":dict(), "back":dict(), "left":dict()}

